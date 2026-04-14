`timescale 1ns / 1ps

module i2c_master(
    input clk,
    input reset,
    input i2c_en,
    input i2c_start,
    input i2c_stop,
    input [7:0] tx_data,
    output logic cm_done,
    output logic tx_ready,
    output logic [7:0] rx_data,
    output logic scl,
    inout sda
);

    logic clear;
    logic [$clog2(500)-1:0] counter;

    clk_counter U_CLK_COUNTER(
        .clk(clk),
        .reset(reset),
        .clear(clear),
        .counter(counter)
    );

    typedef enum 
    {   IDLE, COMMAND,
        START1, START2,
        WRITE1, WRITE2, WRITE3, WRITE4,
        READ1, READ2, READ3, READ4,
        WACK1, WACK2, WACK3, WACK4,
        RACK1, RACK2, RACK3, RACK4,
        STOP1, STOP2 } state_t;

    state_t state, state_next;
    logic sda_r, sda_next;
    logic sel, sel_next;
    logic [3:0] bit_count, bit_count_next;
    logic [7:0] rx_data_next;
    assign sda = sel ? sda_r : 1'bz;
    
    always_ff @(posedge clk or posedge reset ) begin
        if(reset) begin
            state <= IDLE;
            sda_r <= 1;
            sel <= 1;
            bit_count <= 0;
            rx_data <= 0;
        end
        else begin
            state <= state_next;
            sda_r <= sda_next;
            sel <= sel_next;
            bit_count <= bit_count_next;
            rx_data <= rx_data_next;
        end
    end

    always_comb begin
        state_next = state;
        sda_next = sda_r;
        sel_next = sel;
        clear = 0;
        bit_count_next = bit_count;
        cm_done = 0;
        tx_ready = 0;
        scl = 1;
        rx_data_next = rx_data;
        case(state)
            IDLE : begin
                tx_ready = 1;
                scl = 1;
                sda_next = 1;
                sel_next = 1;
                clear = 1;
                bit_count_next = 0;
                if(i2c_en & i2c_start & !i2c_stop) begin
                    state_next = START1;
                end
            end
            START1 : begin
                scl = 1;
                sda_next = 0;
                sel_next = 1;
                if(counter == 499) begin
                    clear = 1;
                    state_next = START2;
                    cm_done = 1;
                end
            end
            START2 : begin
                scl = 0;
                sda_next = 0;
                sel_next = 1;
                if(counter == 499) begin
                    if(!i2c_start & !i2c_stop) begin
                        state_next = WRITE1;
                        clear = 1;
                    end
                    else begin
                        state_next = IDLE;
                    end
                end
            end
            WRITE1 : begin
                scl = 0;
                sda_next = tx_data[7-bit_count];
                sel_next = 1;
                if(counter == 249) begin
                    clear = 1;
                    state_next = WRITE2;
                end
            end
            WRITE2 : begin
                scl = 1;
                sda_next = tx_data[7-bit_count];
                sel_next = 1;
                if(counter == 249) begin
                    clear = 1;
                    state_next = WRITE3;
                end  
            end
            WRITE3 : begin
                scl = 1;
                sda_next = tx_data[7-bit_count];
                sel_next = 1;
                if(counter == 249) begin
                    clear = 1;
                    state_next = WRITE4;
                end 
            end
            WRITE4 : begin
                scl = 0;
                sda_next = tx_data[7-bit_count];
                sel_next = 1;
                if(counter == 249) begin
                    clear = 1;
                    if(bit_count == 7) begin
                        bit_count_next = 0;
                        state_next = WACK1;
                    end
                    else begin
                        bit_count_next = bit_count + 1;
                        state_next = WRITE1;
                    end
                end
            end
            WACK1 : begin
                scl = 0;
                sel_next = 0;
                if(counter == 249) begin
                    clear = 1;
                    state_next = WACK2;
                end
            end
            WACK2 : begin
                scl = 1;
                sel_next = 0;
                if(counter == 249) begin
                    clear = 1;
                    state_next = WACK3;
                end
            end
            WACK3 : begin
                scl = 1;
                sel_next = 0;
                if(counter == 249) begin
                    clear = 1;
                    state_next = WACK4;
                end
            end
            WACK4 : begin
                scl = 0;
                sel_next = 0;
                if(counter == 249) begin
                    clear = 1;
                    state_next = COMMAND;
                    cm_done = 1;
                end
            end
            COMMAND : begin
                clear = 1;
                scl = 0;
                case({i2c_start, i2c_stop})
                    2'b00 : begin
                        state_next = WRITE1;
                    end
                    2'b01 : begin
                        state_next = STOP1;
                    end
                    2'b10 : begin
                        state_next = START1;
                    end
                    2'b11 : begin
                        state_next = READ1;
                    end
                endcase
            end
            READ1 : begin
                scl = 0;
                sel_next = 0;
                if(counter == 249) begin
                    clear = 1;
                    state_next = READ2;
                end
            end
            READ2 : begin
                scl = 1;
                sel_next = 0;
                if(counter == 249) begin
                    clear = 1;
                    state_next = READ3;
                    rx_data_next[7-bit_count] = sda;
                end
            end
            READ3 : begin
                scl = 1;
                sel_next = 0;
                if(counter == 249) begin
                    clear = 1;
                    state_next = READ4;
                end
            end
            READ4 : begin
                scl = 0;
                sel_next = 0;
                if(counter == 249) begin
                    clear = 1;
                    if(bit_count == 7) begin
                        bit_count_next = 0;
                        state_next = RACK1;
                    end
                    else begin
                        bit_count_next = bit_count + 1;
                        state_next = READ1;
                    end
                end
            end
            RACK1 : begin
                scl = 0;
                sda_next = 0;
                sel_next = 1;
                if(counter == 249) begin
                    clear = 1;
                    state_next = RACK2;
                end
            end
            RACK2 : begin
                scl = 1;
                sda_next = 0;
                sel_next = 1;
                if(counter == 249) begin
                    clear = 1;
                    state_next = RACK3;
                end
            end
            RACK3 : begin
                scl = 1;
                sda_next = 0;
                sel_next = 1;
                if(counter == 249) begin
                    clear = 1;
                    state_next = RACK4;
                end
            end
            RACK4 : begin
                scl = 0;
                sel_next = 0;
                sel_next = 1;
                if(counter == 249) begin
                    clear = 1;
                    state_next = COMMAND;
                    cm_done = 1;
                end
            end
            STOP1 : begin
                scl = 1;
                sda_next = 0;
                sel_next = 1;
                if(counter == 499) begin
                    clear = 1;
                    state_next = STOP2;
                end
            end
            STOP2 : begin
                scl = 1;
                sda_next = 1;
                sel_next = 1;
                if(counter == 499) begin
                    clear = 1;
                    state_next = IDLE;
                    cm_done = 1;
                end
            end
        endcase
    end

    

endmodule

module clk_counter(
    input clk,
    input reset,
    input clear,
    output logic [$clog2(500)-1:0] counter
);

    always_ff @(posedge clk or posedge reset) begin
        if(reset) begin
            counter <= 0;
        end
        else begin
            if(clear) begin
                counter <= 0;
            end
            else begin
                counter <= counter + 1;
            end
        end
    end


    
endmodule
