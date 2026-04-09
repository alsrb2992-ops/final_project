# ===================================================================
# Vivado 프로젝트 자동 생성 스크립트
# ===================================================================

# 인자 파싱
if { $argc < 3 } {
    puts "ERROR: Insufficient arguments!"
    puts "Usage: vivado -mode batch -source create_project.tcl -tclargs <project_name> <project_dir> <board_part>"
    exit 1
}

set project_name [lindex $argv 0]
set project_dir  [lindex $argv 1]
set board_part   [lindex $argv 2]

puts "======================================"
puts "Project Configuration:"
puts "  Name: $project_name"
puts "  Dir:  $project_dir"
puts "  Board: $board_part"
puts "======================================"

# 프로젝트 생성
create_project $project_name $project_dir -part xc7z020clg400-1 -force
set_property board_part $board_part [current_project]

# IP Repository 추가 (Custom IP 인식)
if { [file isdirectory ip_repo] } {
    set ip_path [file normalize ip_repo]
    set_property ip_repo_paths $ip_path [current_project]
    update_ip_catalog -rebuild
    puts "\[OK\] Added IP repository: $ip_path"
} else {
    puts "WARNING: ip_repo/ folder not found. Custom IPs may not be available."
}

# RTL 소스 추가
set rtl_files [glob -nocomplain rtl/*.v]
if { [llength $rtl_files] > 0 } {
    add_files -fileset sources_1 $rtl_files
    puts "\[OK\] Added RTL sources ([llength $rtl_files] files)"
} else {
    puts "WARNING: No rtl/*.v files found."
}

# Constraint 추가
set xdc_files [glob -nocomplain const/*.xdc]
if { [llength $xdc_files] > 0 } {
    add_files -fileset constrs_1 $xdc_files
    puts "\[OK\] Added constraint files"
} else {
    puts "WARNING: No const/*.xdc files found."
}

# Block Design 불러오기
if { [file exists bd/ZYNQ_PS_PL.tcl] } {
    source bd/ZYNQ_PS_PL.tcl
    puts "\[OK\] Loaded Block Design"
    
    # Block Design Wrapper 생성 (자동 감지)
    set bd_files [get_files *.bd]
    if { [llength $bd_files] > 0 } {
        set bd_file [lindex $bd_files 0]
        make_wrapper -files $bd_file -top
        
        # Wrapper 파일 찾기 (절대 경로)
        set wrapper_files [glob -nocomplain [file normalize $project_dir]/$project_name.gen/sources_1/bd/*/hdl/*_wrapper.v]
        if { [llength $wrapper_files] > 0 } {
            add_files -norecurse $wrapper_files
            
            # Wrapper 이름 추출
            set wrapper_name [file rootname [file tail [lindex $wrapper_files 0]]]
            set_property top $wrapper_name [current_fileset]
            puts "\[OK\] Created Block Design wrapper: $wrapper_name"
        } else {
            # 대체 경로 (BD 이름 직접 지정)
            set alt_wrapper [file normalize $project_dir/$project_name.gen/sources_1/bd/ZYNQ_PS_PL/hdl/ZYNQ_PS_PL_wrapper.v]
            if { [file exists $alt_wrapper] } {
                add_files -norecurse $alt_wrapper
                set_property top ZYNQ_PS_PL_wrapper [current_fileset]
                puts "\[OK\] Created Block Design wrapper: ZYNQ_PS_PL_wrapper"
            } else {
                puts "WARNING: Wrapper file not found at expected paths."
                puts "  Searched: [file normalize $project_dir]/$project_name.gen/sources_1/bd/*/hdl/*_wrapper.v"
                puts "  Alt path: $alt_wrapper"
            }
        }
    } else {
        puts "WARNING: No Block Design (.bd) file found."
    }
} else {
    puts "WARNING: bd/ZYNQ_PS_PL.tcl not found."
}

puts ""
puts "======================================"
puts "\[SUCCESS\] Project created!"
puts "======================================"
puts "Open project:"
puts "  vivado $project_dir/$project_name.xpr"
puts ""
