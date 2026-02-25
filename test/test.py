import cocotb
from cocotb.triggers import ClockCycles

@cocotb.test()
async def test_project(dut):
    dut._log.info("Start")

    dut._log.info("Wait for reset deassertion")
    for _ in range(2000):
        if int(dut.rst_n.value) == 1:
            break
        await ClockCycles(dut.clk, 1)
    assert int(dut.rst_n.value) == 1

    dut._log.info("Observe TB progress")
    for _ in range(20000):
        if int(dut.fail_cnt.value) != 0:
            raise AssertionError("Verilog TB reported fail_cnt != 0")
        if int(dut.pass_cnt.value) > 0:
            break
        await ClockCycles(dut.clk, 1)

    assert int(dut.fail_cnt.value) == 0
    assert int(dut.pass_cnt.value) > 0

    return
