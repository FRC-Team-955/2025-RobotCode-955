package frc.robot;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class UtilTests {
    @Test
    void findArrayIndexWithClosestValueTest() {
        double[] array = {0.5, 3, 1.342, 5.321, -1.23, -0.5, -3};
        assertEquals(
                1.342,
                array[Util.findArrayIndexWithClosestValue(1, array)]
        );
        assertEquals(
                3,
                array[Util.findArrayIndexWithClosestValue(2.4, array)]
        );
        assertEquals(
                5.321,
                array[Util.findArrayIndexWithClosestValue(7, array)]
        );
        assertEquals(
                -1.23,
                array[Util.findArrayIndexWithClosestValue(-2, array)]
        );
        assertEquals(
                -3,
                array[Util.findArrayIndexWithClosestValue(-2.4, array)]
        );
        assertEquals(
                0.5,
                array[Util.findArrayIndexWithClosestValue(0.5, array)]
        );
    }
}
