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

//Matt's Code Below:
/*    private boolean outputStateDebounced = false;

    private double lastTime = 0;
    private double debounceTime = 500;

    public boolean toggleButtonDebounced(boolean inputButton){
        double currentTime = System.currentTimeMillis();
        if(inputButton && currentTime > (lastTime+debounceTime)){
            outputStateDebounced = !outputStateDebounced;
            lastTime = currentTime;
        }
        return outputStateDebounced;
    }*/

}
