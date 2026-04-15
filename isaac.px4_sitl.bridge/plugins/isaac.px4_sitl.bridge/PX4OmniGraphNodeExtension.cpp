#define CARB_EXPORTS

#include <carb/PluginUtils.h>

#include <omni/kit/IApp.h>
#include <omni/graph/core/IGraphRegistry.h>
#include <omni/graph/core/ogn/Database.h>
#include <omni/graph/core/ogn/Registration.h>

#include "MavlinkServerManager.hpp"
#include "Px4ProcessManager.hpp"

const struct carb::PluginImplDesc pluginImplDesc = { 
    "isaac.px4_sitl.bridge.plugin",
    "A PX4 Bridge for Isaac Sim", "UoSM",
    carb::PluginHotReload::eEnabled, "dev" 
};

CARB_PLUGIN_IMPL_DEPS(omni::graph::core::IGraphRegistry,
                      omni::fabric::IPath,
                      omni::fabric::IToken,
                      omni::kit::IApp)

DECLARE_OGN_NODES()

namespace isaac {
namespace px4_sitl {
namespace bridge {

class PX4OmingraphNodeExtension : public omni::ext::IExt
{
public:
    void onStartup(const char *extId) override{
        printf("PX4 bridge extension startup..... (ext_id: %s).\n", extId);
        INITIALIZE_OGN_NODES()
    }

    void onShutdown() override{
        printf("PX4 bridge extension shutdown\n");
        auto &processManager = Px4ProcessManager::getInstance();
        auto &serverManager = MavlinkServerManager::getInstance();
        processManager.terminateAll();
        serverManager.removeAllConnections();
        RELEASE_OGN_NODES()
    }

private:
};

}
}
}


CARB_PLUGIN_IMPL(pluginImplDesc, isaac::px4_sitl::bridge::PX4OmingraphNodeExtension)

void fillInterface(isaac::px4_sitl::bridge::PX4OmingraphNodeExtension& iface)
{
}