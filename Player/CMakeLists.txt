set (TARGET_NAME Urho3DPlayer)
# Define source files

define_source_files (GLOB_CPP_PATTERNS *.cpp GLOB_H_PATTERNS *.h)

setup_main_executable ()
target_link_libraries(Urho3DPlayer Urho3DPhysX)