package org.firstinspires.ftc.teamcode;

public class Toggle {
    private boolean outputState = false;

    public boolean toggleButton(boolean inputButton) {
        if (inputButton == true){
            if (outputState == true){
                outputState = false;
            }
            else{
                outputState = true;
            }
        }
        return outputState;
    }

}
