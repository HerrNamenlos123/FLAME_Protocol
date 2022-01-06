
-- Utility functions
function appendTable(tableA, tableB)
    for _,v in ipairs(tableB) do 
        table.insert(tableA, v) 
    end
end



-- Main library project
project "FLAME_Protocol"
    kind "StaticLib"
    language "C++"
    cppdialect "C++17"
    staticruntime "on"
    location "build"
    targetname "FLAME_Protocol"
    targetdir "bin/%{cfg.buildcfg}"
    --system "Windows"
    --architecture "x86_64"

    filter "configurations:Debug"
        defines { "DEBUG", "_DEBUG", "NDEPLOY" }
        runtime "Debug"
        symbols "On"
        optimize "Off"

    filter "configurations:Release"
        defines { "NDEBUG", "NDEPLOY" }
        runtime "Release"
        symbols "On"
        optimize "On"

    filter "configurations:Deploy"
        defines { "NDEBUG", "DEPLOY" }
        runtime "Release"
        symbols "Off"
        optimize "On"

    filter {}


    -- Include directories
    local _includedirs = { 
        _SCRIPT_DIR .. "/src"
    }
    includedirs (_includedirs)

    
    -- Main source files
    files ({ "src/**" })

    
    -- Include and linker information for premake projects using this library
    FLAMEPROTOCOL_INCLUDE_DIRS = {}
    appendTable(FLAMEPROTOCOL_INCLUDE_DIRS, _includedirs)

    FLAMEPROTOCOL_LINK_DIRS = {}
    appendTable(FLAMEPROTOCOL_LINK_DIRS, _SCRIPT_DIR .. "/bin/%{cfg.buildcfg}/")

    FLAMEPROTOCOL_LINKS = { "FLAME_Protocol" }
