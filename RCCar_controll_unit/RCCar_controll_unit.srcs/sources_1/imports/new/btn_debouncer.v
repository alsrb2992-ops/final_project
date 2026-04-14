`timescale 1ns / 1ps

module btn_debouncer #(parameter DEBOUNCE_LIMIT = 20'd999_999) (
    input      clk,
    input      reset_n,
    input      noisy_btn,  // raw noisy button input
    output     btn_edge
);
    reg [19:0] count;
    reg btn_state = 0;
    
    reg clean_btn;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            count <= 0;
            btn_state <= 0;
            clean_btn <= 0;
        end 
        else if (noisy_btn == btn_state) begin 
            count <= 0;
        end
        else begin
            if (count < DEBOUNCE_LIMIT)  
                count <= count + 1;
            else begin  
                btn_state <= noisy_btn;
                clean_btn <= noisy_btn;
                count <= 0;  
            end
        end
    end

    reg clean_btn_delay;

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            clean_btn_delay <= 0;
        end    
        else begin
            clean_btn_delay <= clean_btn;
        end
    end
    
    assign btn_edge = clean_btn & !clean_btn_delay;

endmodule