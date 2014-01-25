set (INC 
	src
)

set (INC_SYS
)

set (SRC 
	src/c_loops.cc
    src/cell.cc
    src/cmd_line.cc
    src/common.cc
    src/container_prd.cc
    src/container.cc
    src/pre_container.cc
    src/unitcell.cc
    src/v_base.cc
    src/v_compute.cc
#    src/voro++.cc
    src/wall.cc
    src/c_interface.cc
    
    src/c_loops.hh
    src/cell.hh
    src/common.hh
    src/config.hh
    src/container_prd.hh
    src/container.hh
    src/pre_container.hh
    src/rad_option.hh
    src/unitcell.hh
    src/v_base.hh
    src/v_compute.hh
#    src/voro++.hh
    src/wall.hh
    src/worklist.hh
    src/c_interface.hh
)

blender_add_lib(extern_voro "${SRC}" "${INC}" "${INC_SYS}")
