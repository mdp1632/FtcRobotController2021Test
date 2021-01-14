package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Toggle {
    private boolean outputState = false;

    public boolean toggleBoolean(boolean InputBool){
        if (InputBool){
            if (outputState) {
                outputState = false;
            } else {
                outputState = true;
            }
        }
        return outputState;
    }
}

// TODO: 11/29/2020 add timer for debouncing.
/*
public class Toggle {
    private boolean outputState = false;
    // private double lastTime = 0;
    // private double debounceTime = 250;      //in milliseconds

    public boolean toggleBoolean(boolean InputBool){

        double currentTime = System.currentTimeMillis();
        if (InputBool && (currentTime > (lastTime + debounceTime))){
        if (InputBool){
            if (outputState) {
                outputState = false;
            } else {
                outputState = true;
            }
        }
        lastTime = System.currentTimeMillis();
        return outputState;

    }

}
*/