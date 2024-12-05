module testbench();

  logic        CLK;
  logic        Reset;

  logic [31:0] WriteData, DataAdr;
  logic        MemWrite;

  // instantiate device to be tested
  top dut(CLK, Reset, WriteData, DataAdr, MemWrite);
  
  // initialize test
  initial
    begin
      Reset <= 1; # 22; Reset <= 0;
    end

  // generate clock to sequence tests
  always
    begin
      CLK <= 1; # 5; CLK <= 0; # 5;
    end

  // check results
  always @(negedge CLK)
    begin
      if(MemWrite) begin
        if(DataAdr === 84 & WriteData === 71) begin
          $display("Simulation succeeded");
          $stop;
        end else if (DataAdr !== 80) begin
          $display("Simulation failed");
          $stop;
        end
      end
    end
endmodule