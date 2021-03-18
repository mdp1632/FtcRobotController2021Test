
package org.firstinspires.ftc.teamcode;

import android.renderscript.ScriptGroup;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ToggleMdP {
    private boolean outputState = false;

    private double lastTime = 0;
    private double lastTime2 = 0;

    private double debounceTime = 250;


    public boolean toggleBoolean(boolean InputBool) {
        if (InputBool) {
            if (outputState) {
                outputState = false;
            } else {
                outputState = true;
            }
        }
        return outputState;
    }

    public boolean toggleBooleanDebounced(boolean InputBool){
        double currentTime = System.currentTimeMillis();

        if(InputBool && (currentTime > (lastTime + debounceTime))){
            outputState = !outputState;
            lastTime = currentTime;
        }
        return outputState;
    }

    public boolean toggleBooleanDebounced2(boolean InputBool) {
        double currentTime = System.currentTimeMillis();

        if (InputBool && (currentTime > (lastTime2 + debounceTime))){
            if (outputState) {
                outputState = false;
            } else {
                outputState = true;
            }
            lastTime2 = currentTime;
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