# create_project.tcl
# -force로 기존 프로젝트 덮어쓰기 (기존 .srcs 폴더 유지됨)
create_project -force top D:/KJW/final_project/top -part xc7a35tcpg236-1

# 기존 .srcs 폴더 안의 모든 .sv 파일 자동으로 추가
add_files [glob D:/KJW/final_project/top/top.srcs/sources_1/new/*.sv]

# 만약 constraints 폴더에 .xdc 파일이 있다면
add_files -fileset constrs_1 [glob D:/KJW/final_project/top/top.srcs/constrs_1/new/*.xdc]

# Top 모듈 설정 (실제 top 모듈 파일 이름으로 변경)
set_property top top_module_name [current_fileset]

update_compile_order -fileset sources_1