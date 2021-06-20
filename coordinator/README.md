# The Coordinator Folder

This is the coordinator folder area. This folder hosts all the coordinator related objects:

    Coordinator Folder
    ├── bringup_coordinator                             # Coordinator BG Launcher Package
    │   └── bringup_coordinator.launch                  # Launcher File
    ├── coordinator                                     # Coordinator Code Package
    │   ├── CMakeLists.txt                              # CMake List
    │   ├── package.xml                                 # Package XML
    │   └── src                                         # Source Folder
    │       ├── arm_test.cpp                            # simple arm grip motion test
    │       ├── coordinator.cpp                         # OBSOLETE package (Original Coordinator Example)
    │       ├── grab_tote_v2.cpp                        # LATEST grab FAKE_TOTE and REAL_TOTE
    │       ├── grab_tote.cpp                           # OBSOLETE PACKAGE (Original grab_tote)
    │       └── visit_all.cpp                           # LATEST command the vehicle to go all station orderly
    └── README.md                                       # This File!

## Command Related to Coordinator

### Latest Command

1. **grab_tote_v2:** `rosrun coordinator grab_tote_v2`
2. **visit_all:** `rosrun coordinator visit_all`

### Unused Command

1. **grab_tote:** `rosrun coordinator grab_tote`
2. **arm_test:** `rosrun coordinator arm_test`

### Unimplemented Command

1. **bringup_coordinator:** `roslaunch bringup_coordinator bringup_coordinator.launch`
