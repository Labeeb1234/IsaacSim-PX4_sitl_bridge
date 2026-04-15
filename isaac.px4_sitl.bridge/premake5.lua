-- Setup the basic extension variables
local ext = get_current_extension_info()
local project_name = "isaac/px4_sitl/bridge"

-- Set up the basic shared project information
project_ext(ext)

-- --------------------------------------------------------------------------------------------------------------
-- Helper variable containing standard configuration information for projects containing OGN files.
local ogn = get_ogn_project_information(ext, project_name)

-- --------------------------------------------------------------------------------------------------------------
-- Link folders and files that should be packaged with the extension.
repo_build.prebuild_link {
    { "data", ext.target_dir.."/data" },
    { "docs", ext.target_dir.."/docs" },
    { "include", ext.target_dir.."/include" }
    -- {"/home/inlabust/labeeb/drone_isaacsim/PX4-Autopilot/build/px4_sitl_default/px4"}
}

--------------------------------------------------------------------------------------------------------------
-- Breaking this out as a separate project ensures the .ogn files are processed before their results are needed.
project_ext_ogn( ext, ogn )

--------------------------------------------------------------------------------------------------------------
-- Build the C++ plugin that will be loaded by the extension.
project_ext_plugin(ext, ogn.plugin_project)
    add_files("source", "plugins/"..ogn.module)
    add_files("nodes", "plugins/nodes")
    -- Add the standard dependencies all OGN projects have; includes, libraries to link, and required compiler flags
    add_ogn_dependencies(ogn)

    -- Begin OpenUSD
    extra_usd_libs = {
        "usdPhysics",
        "usdUtils",
    }


    -- INCLUDE PATHS (based on YOUR structure)
    includedirs {
        ext.target_dir.."/include",
        "/home/inlabust/labeeb/drone_isaacsim/PX4-Autopilot/build/px4_sitl_default/mavlink"
    }

    cppdialect "C++17"
    runtime "Release"
    rtti "On"

    filter { "system:linux" }
        exceptionhandling "On"
        staticruntime "Off"
        includedirs {
            "%{target_deps}/python/include/python3.10",
            "%{target_deps}/cuda"
        }
        -- NOTE: std::string not ABI safe in omni/carbonite context
        -- see https://docs.omniverse.nvidia.com/kit/docs/carbonite/latest/docs/omni/String.html
        buildoptions { "-D_GLIBCXX_USE_CXX11_ABI=0 -pthread -lstdc++fs -Wno-error -fabi-version=11" }
        linkoptions { "-Wl,--disable-new-dtags -Wl,-rpath,%{target_deps}/python/lib:" }