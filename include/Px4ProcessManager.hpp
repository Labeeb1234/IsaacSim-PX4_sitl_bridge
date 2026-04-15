#pragma once
#include <omni/kit/IApp.h>
#include <omni/ext/IExt.h>
#include <omni/ext/ExtensionsUtils.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <set>
#include <string>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <filesystem>

namespace isaac {
namespace px4_sitl {
namespace bridge {

static std::string getExtPath(void)
{
    omni::kit::IApp *app = carb::getFramework()->acquireInterface<omni::kit::IApp>();
    auto ext_manager = app->getExtensionManager();
    auto extId = omni::ext::getEnabledExtensionId(ext_manager, "isaac.px4_sitl.bridge");
    return omni::ext::getExtensionPath(ext_manager, extId);
}

static std::string getBinaryPath(const std::string &envVar)
{
    const char *envPath = std::getenv(envVar.c_str());
    if (envPath != nullptr && std::filesystem::exists(envPath))
    {
        return std::string(envPath);
    }
    return "";
}

class Px4ProcessManager
{
public:
    static Px4ProcessManager &getInstance()
    {
        static Px4ProcessManager instance;
        return instance;
    }

    pid_t launch_px4(const int id = 0, const std::string &model = "isaacsim_s500")
    {
        std::lock_guard<std::mutex> lock(mtx_);
        const std::string ext_path = getExtPath();
        setenv("PATH", (std::string(getenv("PATH")) + ":" + ext_path + "/px4/bin").c_str(), 1);        
        std::string px4_bin = ext_path + "/px4/bin/px4";
        std::string startup_file = ext_path + "/px4/etc/init.d-posix/rcS";
        std::string rootfs_directory = ext_path + "/px4/ROMFS/px4fmu_common";
        std::string working_directory = ext_path + "/px4/";
        // Override with user-defined PX4_BINARY_PATH
        const std::string px4_binary_path = getBinaryPath("PX4_BINARY_PATH");
        if (!px4_binary_path.empty() && std::filesystem::exists(px4_binary_path + "/bin/px4"))
        {            
            setenv("PATH", (std::string(getenv("PATH")) + ":" + px4_binary_path).c_str(), 1);
            px4_bin = px4_binary_path + "/bin/px4";
            startup_file = px4_binary_path + "/etc/init.d-posix/rcS";
            rootfs_directory = px4_binary_path + "/ROMFS/px4fmu_common";
            working_directory = px4_binary_path + "/";
        }
        // if user did not set PX4_SIM_MODEL environment variable
        if (std::getenv("PX4_SIM_MODEL") == nullptr)
        {
            setenv("PX4_SIM_MODEL", model.c_str(), 1);
        }
        std::string instance = std::to_string(id);
        const char *argv[] = {
            px4_bin.c_str(),
            "-s", startup_file.c_str(),
            "-w", working_directory.c_str(),
            "-i", instance.c_str(),
            "-d", rootfs_directory.c_str(),
            nullptr
        };

        pid_t pid = forkAndExecute(px4_bin, argv);
        if (pid > 0)
        {
            registerProcess(pid, IS_GROUP);
            return pid;
        }
        return 0;
    }

    void launch_uxrce_agent(void)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        std::string uxrce_agent_bin = "";

        // Override with user-defined UXRCE_BINARY_PATH
        const std::string uxrce_binary_path = getBinaryPath("UXRCE_BINARY_PATH");
        if (!uxrce_binary_path.empty())
        {
            uxrce_agent_bin = uxrce_binary_path + "/MicroXRCEAgent";
        }
        else
        {
            const std::string ext_path = getExtPath();
            uxrce_agent_bin = ext_path + "/uxrce/MicroXRCEAgent";
            // Set LD_LIBRARY_PATH
            const char *current_ld_library_path = getenv("LD_LIBRARY_PATH");
            std::string ld_library_path = ext_path + "/uxrce/temp_install/fastdds-3.1/lib:" +
                                          ext_path + "/uxrce/temp_install/fastcdr-2.2.4/lib:" +
                                          ext_path + "/uxrce/temp_install/microxrcedds_client-3.0.0/lib:" +
                                          ext_path + "/uxrce/temp_install/microcdr-2.0.1/lib";

            if (current_ld_library_path != nullptr)
            {
                ld_library_path = std::string(current_ld_library_path) + ":" + ld_library_path;
            }

            if (setenv("LD_LIBRARY_PATH", ld_library_path.c_str(), 1) != 0)
            {
                CARB_LOG_ERROR("Failed to set LD_LIBRARY_PATH");
                return;
            }
        }

        if (!std::filesystem::exists(uxrce_agent_bin))
        {
            CARB_LOG_ERROR("MicroXRCEAgent binary not found, please set UXCRE_BINARY_PATH environment variable");
            return;
        }

        const char *argv[] = {
            "MicroXRCEAgent",
            "udp4",
            "-p",
            "8888",
            (char *)nullptr};
        pid_t pid = forkAndExecute(uxrce_agent_bin, argv);
        if (pid > 0)
        {
            registerProcess(pid, IS_STANDALONE);
            isUxrceRunning_ = true;
        }
    }

    bool terminate_process(pid_t pid, const bool is_group = true)
    {
        if (pid <= 0)
            return true;

        if (is_group)
        {
            // Kill the process group
            if (killpg(pid, SIGTERM) == 0)
            {
                auto start = std::chrono::steady_clock::now();
                while (killpg(pid, 0) == 0)
                {
                    if (std::chrono::steady_clock::now() - start > TERMINATION_TIMEOUT)
                    {
                        killpg(pid, SIGKILL);
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                managed_processes_.erase(pid);
                return true;
            }
        }
        else
        {
            // Kill the individual process
            if (kill(pid, SIGTERM) == 0)
            {
                auto start = std::chrono::steady_clock::now();
                while (kill(pid, 0) == 0)
                {
                    if (std::chrono::steady_clock::now() - start > TERMINATION_TIMEOUT)
                    {
                        kill(pid, SIGKILL);
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                managed_processes_.erase(pid);
                return true;
            }
        }

        // All else failed, force kill
        if (is_group)
            killpg(pid, SIGKILL);
        else
            kill(pid, SIGKILL);

        managed_processes_.erase(pid);
        return false;
    }

    void terminateAll()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!managed_processes_.empty())
        {
            std::map<pid_t, bool> copy = managed_processes_;
            for (const auto &[pid, type] : copy)
            {
                terminate_process(pid, type);
            }
            managed_processes_.clear();
            isUxrceRunning_ = false;
        }
    }

    bool isUxRCEAgentRunning(void)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return isUxrceRunning_;
    }

private:
    // Delete copy constructor and assignment operator
    Px4ProcessManager(const Px4ProcessManager &) = delete;
    Px4ProcessManager &operator=(const Px4ProcessManager &) = delete;
    Px4ProcessManager()
    {
        setupSignalHandlers();
    }

    ~Px4ProcessManager()
    {
        terminateAll();
    }

    static void handleSignal(int signum)
    {
        if (signum == SIGCHLD)
        {
            int status;
            pid_t pid;
            while ((pid = waitpid(-1, &status, WNOHANG)) > 0)
            {
                if (WIFEXITED(status))
                    CARB_LOG_INFO("Process %d exited with status %d", pid, WEXITSTATUS(status));
                else if (WIFSIGNALED(status))
                    CARB_LOG_INFO("Process %d terminated by signal %d", pid, WTERMSIG(status));
            }
        }
    }

    void registerProcess(pid_t pid, bool is_group)
    {
        managed_processes_.insert({pid, is_group});
    }

    pid_t forkAndExecute(const std::string &binary, const char *argv[])
    {
        validatePath(binary);
        pid_t pid = fork();
        if (pid == -1)
            throw std::runtime_error("Failed to fork");

        if (pid == 0)
        {
            setpgid(0, 0); // Set process group for the child
            if (execv(binary.c_str(), const_cast<char *const *>(argv)) == -1)
            {
                CARB_LOG_ERROR("Failed to execute %s, err %s", binary.c_str(), strerror(errno));
                exit(1);
            }
        }
        setpgid(pid, pid);
        return pid;
    }

    void setupSignalHandlers()
    {
        struct sigaction sa;
        sa.sa_handler = Px4ProcessManager::handleSignal;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = SA_RESTART | SA_NOCLDSTOP;

        if (sigaction(SIGCHLD, &sa, nullptr) == -1)
            throw std::runtime_error("Failed to set up signal handler");
    }

    void validatePath(const std::string &path)
    {
        if (access(path.c_str(), F_OK) == -1)
            throw std::runtime_error("Invalid path: " + path);
    }

    std::map<pid_t, bool> managed_processes_;
    static constexpr std::chrono::milliseconds TERMINATION_TIMEOUT{2000};
    static constexpr bool IS_GROUP{1};
    static constexpr bool IS_STANDALONE{0};
    std::mutex mtx_;
    std::atomic<bool> isUxrceRunning_{false};
};

}
}
} 