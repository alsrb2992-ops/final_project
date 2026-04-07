// ============================================================
// tb_packet_parser.sv
// packet_parser 단독 CS 검증 테스트벤치
// ============================================================
`timescale 1ns / 1ps

module tb_packet_parser;

    localparam CLK_PERIOD = 20;  // 50MHz

    logic          clk;
    logic          rst_n;
    logic   [ 7:0] byte_in;
    logic          byte_valid;
    logic          pkt_start;

    logic          ct_start_bit;
    logic   [ 7:0] lsn;
    logic   [15:0] fsa_raw;
    logic   [15:0] lsa_raw;
    logic   [15:0] cs_rx;
    logic   [15:0] si_raw;
    logic          si_valid;
    logic          pkt_done;
    logic          cs_ok;

    // ============================================================
    // Scoreboard
    // ============================================================
    integer        total_checks;
    integer        pass_count;
    integer        fail_count;

    task sb_init;
        total_checks = 0;
        pass_count   = 0;
        fail_count   = 0;
    endtask

    task sb_check(input integer test_num, input string test_desc,
                  input logic actual, input logic expected,
                  input string signal_name);
        total_checks = total_checks + 1;
        if (actual === 1'bx || actual === 1'bz) begin
            fail_count = fail_count + 1;
            $display("  [UNKN] T%0d | %s | %s=x/z (exp:%b) <<<", test_num,
                     test_desc, signal_name, expected);
        end else if (actual === expected) begin
            pass_count = pass_count + 1;
            $display("  [PASS] T%0d | %s | %s=%b", test_num, test_desc,
                     signal_name, actual);
        end else begin
            fail_count = fail_count + 1;
            $display("  [FAIL] T%0d | %s | %s=%b (exp:%b) <<<", test_num,
                     test_desc, signal_name, actual, expected);
        end
    endtask

    task sb_report;
        $display("========================================");
        $display("  SCOREBOARD REPORT");
        $display("  Total : %0d", total_checks);
        $display("  PASS  : %0d", pass_count);
        $display("  FAIL  : %0d", fail_count);
        if (fail_count == 0) $display("  Result: ALL PASS");
        else $display("  Result: %0d FAILED", fail_count);
        $display("========================================");
    endtask

    // ============================================================
    // DUT
    // ============================================================
    packet_parser dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .byte_in     (byte_in),
        .byte_valid  (byte_valid),
        .pkt_start   (pkt_start),
        .ct_start_bit(ct_start_bit),
        .lsn         (lsn),
        .fsa_raw     (fsa_raw),
        .lsa_raw     (lsa_raw),
        .cs_rx       (cs_rx),
        .si_raw      (si_raw),
        .si_valid    (si_valid),
        .pkt_done    (pkt_done),
        .cs_ok       (cs_ok)
    );

    always #(CLK_PERIOD / 2) clk = ~clk;

    // ============================================================
    // 바이트 전송 task (1클럭에 1바이트)
    // ============================================================
    task send_byte_to_parser(input logic [7:0] data);
        @(posedge clk);
        byte_in    = data;
        byte_valid = 1'b1;
        @(posedge clk);
        byte_valid = 1'b0;
    endtask

    // pkt_start 펄스
    task trigger_pkt_start;
        @(posedge clk);
        pkt_start = 1'b1;
        @(posedge clk);
        pkt_start = 1'b0;
    endtask

    // 패킷 전송 (AA55 감지 후 pkt_start 가 왔다고 가정)
    // LSN=1 고정, Si 1개
    task send_packet(input logic is_start, input logic [15:0] fsa_in,
                     input logic [15:0] lsa_in,
                     input logic [15:0] cs_in,  // 직접 지정
                     input logic [15:0] si_in);
        logic [7:0] ct_val;
        ct_val = is_start ? 8'h01 : 8'h00;

        trigger_pkt_start();
        send_byte_to_parser(ct_val);  // CT
        send_byte_to_parser(8'h01);  // LSN=1
        send_byte_to_parser(fsa_in[7:0]);  // FSA LSB
        send_byte_to_parser(fsa_in[15:8]);  // FSA MSB
        send_byte_to_parser(lsa_in[7:0]);  // LSA LSB
        send_byte_to_parser(lsa_in[15:8]);  // LSA MSB
        send_byte_to_parser(cs_in[7:0]);  // CS LSB
        send_byte_to_parser(cs_in[15:8]);  // CS MSB
        send_byte_to_parser(si_in[7:0]);  // Si LSB
        send_byte_to_parser(si_in[15:8]);  // Si MSB

        // pkt_done 대기
        // @(posedge clk);
        // @(posedge clk);
    endtask

    // CS 올바른 값 계산
    // CS = PH ^ FSA ^ {LSN,CT} ^ LSA ^ Si
    function automatic logic [15:0] calc_cs(
        input logic is_start, input logic [15:0] fsa_in,
        input logic [15:0] lsa_in, input logic [15:0] si_in);
        logic [ 7:0] ct_val;
        logic [15:0] cs;
        ct_val  = is_start ? 8'h01 : 8'h00;
        cs      = 16'h55AA;  // PH
        cs      = cs ^ fsa_in;  // FSA
        cs      = cs ^ {8'h01, ct_val};  // {LSN=1, CT}
        cs      = cs ^ lsa_in;  // LSA
        cs      = cs ^ si_in;  // Si
        calc_cs = cs;
    endfunction

    // ============================================================
    // Test variables
    // ============================================================
    integer test_num;
    logic [15:0] t_fsa, t_lsa, t_si;
    logic [15:0] t_cs_ok, t_cs_bad;
    string t_desc;

    // ============================================================
    // Test
    // ============================================================
    initial begin
        clk        = 0;
        rst_n      = 0;
        byte_in    = 0;
        byte_valid = 0;
        pkt_start  = 0;
        test_num   = 0;

        sb_init();

        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        $display("========================================");
        $display("  packet_parser CS verification TB");
        $display("========================================");

        // ------------------------------------------------
        // Test 1: 올바른 CS → cs_ok=1
        // ------------------------------------------------
        test_num = test_num + 1;
        t_fsa    = 16'h0281;  // ~5도
        t_lsa    = 16'h0281;
        t_si     = 16'h04B0;  // 300mm IS=0
        t_cs_ok  = calc_cs(1'b0, t_fsa, t_lsa, t_si);

        $display("--- T%0d: Correct CS -> cs_ok=1 ---", test_num);
        $display("    FSA=0x%04X LSA=0x%04X SI=0x%04X CS=0x%04X", t_fsa, t_lsa,
                 t_si, t_cs_ok);

        send_packet(1'b0, t_fsa, t_lsa, t_cs_ok, t_si);
        sb_check(test_num, "Correct CS", cs_ok, 1'b1, "cs_ok  ");
        sb_check(test_num, "Correct CS", pkt_done, 1'b1, "pkt_done");

        // ------------------------------------------------
        // Test 2: 틀린 CS → cs_ok=0
        // ------------------------------------------------
        test_num = test_num + 1;
        t_cs_bad = t_cs_ok ^ 16'hFFFF;  // 전체 반전

        $display("--- T%0d: Wrong CS (inverted) -> cs_ok=0 ---", test_num);
        $display("    CS_wrong=0x%04X (correct=0x%04X)", t_cs_bad, t_cs_ok);

        send_packet(1'b0, t_fsa, t_lsa, t_cs_bad, t_si);
        sb_check(test_num, "Wrong CS inverted", cs_ok, 1'b0, "cs_ok  ");
        sb_check(test_num, "Wrong CS inverted", pkt_done, 1'b1, "pkt_done");

        // ------------------------------------------------
        // Test 3: CS=0x0000 → cs_ok=0
        // ------------------------------------------------
        test_num = test_num + 1;
        $display("--- T%0d: CS=0x0000 -> cs_ok=0 ---", test_num);

        send_packet(1'b0, t_fsa, t_lsa, 16'h0000, t_si);
        sb_check(test_num, "CS=0x0000", cs_ok, 1'b0, "cs_ok  ");

        // ------------------------------------------------
        // Test 4: CS=0xFFFF → cs_ok=0
        // ------------------------------------------------
        test_num = test_num + 1;
        $display("--- T%0d: CS=0xFFFF -> cs_ok=0 ---", test_num);

        send_packet(1'b0, t_fsa, t_lsa, 16'hFFFF, t_si);
        sb_check(test_num, "CS=0xFFFF", cs_ok, 1'b0, "cs_ok  ");

        // ------------------------------------------------
        // Test 5: 시작 패킷(CT=1) 올바른 CS → cs_ok=1
        // ------------------------------------------------
        test_num = test_num + 1;
        t_cs_ok  = calc_cs(1'b1, t_fsa, t_lsa, t_si);

        $display("--- T%0d: Start packet correct CS -> cs_ok=1 ---", test_num);
        $display("    CS=0x%04X", t_cs_ok);

        send_packet(1'b1, t_fsa, t_lsa, t_cs_ok, t_si);
        sb_check(test_num, "Start pkt correct CS", cs_ok, 1'b1, "cs_ok  ");

        // ------------------------------------------------
        // Test 6: 다른 FSA/LSA 조합 → CS 올바르게 계산
        // ------------------------------------------------
        test_num = test_num + 1;
        t_fsa    = 16'hAA01;  // 340도 근처 (0xAA 포함)
        t_lsa    = 16'hAA01;
        t_si     = 16'h04B0;
        t_cs_ok  = calc_cs(1'b0, t_fsa, t_lsa, t_si);

        $display(
            "--- T%0d: FSA=0xAA01 (contains 0xAA) correct CS -> cs_ok=1 ---",
            test_num);
        $display("    CS=0x%04X", t_cs_ok);

        send_packet(1'b0, t_fsa, t_lsa, t_cs_ok, t_si);
        sb_check(test_num, "FSA=0xAA01 correct CS", cs_ok, 1'b1, "cs_ok  ");

        // ------------------------------------------------
        // Test 7: 1비트만 틀린 CS → cs_ok=0
        // ------------------------------------------------
        test_num = test_num + 1;
        t_fsa    = 16'h0281;
        t_lsa    = 16'h0281;
        t_si     = 16'h04B0;
        t_cs_ok  = calc_cs(1'b0, t_fsa, t_lsa, t_si);
        t_cs_bad = t_cs_ok ^ 16'h0001;  // 1비트만 반전

        $display("--- T%0d: CS off by 1 bit -> cs_ok=0 ---", test_num);
        $display("    CS_correct=0x%04X CS_wrong=0x%04X", t_cs_ok, t_cs_bad);

        send_packet(1'b0, t_fsa, t_lsa, t_cs_bad, t_si);
        sb_check(test_num, "CS off by 1bit", cs_ok, 1'b0, "cs_ok  ");

        // ------------------------------------------------
        // Test 8: si_valid 가 cs_ok 와 같이 나오는지 확인
        // 올바른 CS → si_valid=1
        // ------------------------------------------------
        test_num = test_num + 1;
        t_cs_ok  = calc_cs(1'b0, t_fsa, t_lsa, t_si);
        $display("--- T%0d: si_valid check with correct CS ---", test_num);

        send_packet(1'b0, t_fsa, t_lsa, t_cs_ok, t_si);
        sb_check(test_num, "si_valid correct CS", cs_ok, 1'b1, "cs_ok  ");
        sb_check(test_num, "si_valid correct CS", si_valid, 1'b1, "si_valid");
        // si_valid 는 Si 수신 시 1클럭 펄스 → pkt_done 전 클럭에 발생
        // pkt_done 시점에는 이미 0으로 돌아옴 (정상)

        sb_report();
        $finish;
    end

    initial begin
        $monitor("[%0t] valid=%b start=%b | cs_ok=%b done=%b | si_v=%b", $time,
                 byte_valid, pkt_start, cs_ok, pkt_done, si_valid);
    end

endmodule
