// ============================================================
// round_detector.sv
// CT[bit0]=1 && pkt_start → round_done 펄스 생성
// 순수 조합논리 (CL)
// ============================================================
module round_detector (
    input wire pkt_start,    // packet_sync 의 pkt_start 펄스
    input wire ct_start_bit, // packet_parser 의 ct_start_bit

    output wire round_done  // 1클럭 펄스: 1회전 완료
);

    assign round_done = pkt_start & ct_start_bit;

endmodule
