package org.firstinspires.ftc.teamcode;

public class Toggle {
    private boolean outputState = false;

    public boolean toggleBoolean(boolean InputBool){
        if(InputBool){
            if(outputState){
                outputState = false;
            }
            else{
                outputState = true;
            }
        }
        return outputState;
    }

}

// TODO: 11/29/2020 add timer for debouncing.