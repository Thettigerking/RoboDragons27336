package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.BlueCloseAuto;

import java.util.Objects;

public class Shooting {
    public double outtake(double sspeedx, double sspeed, String shoottype, double RightCurrVeloc) {
        BlueCloseAuto farauto = new BlueCloseAuto();

        if (Objects.equals(shoottype, "CLOSE")) {
            if (RightCurrVeloc > sspeed + 25) {
                return (0);
            } else if (RightCurrVeloc < sspeed - 25) {
                return (sspeed + (sspeed * sspeedx));
            } else {
                return (sspeed);
            }
        } else if (Objects.equals(shoottype, "REDFAR")){
            if (RightCurrVeloc > sspeed + 100) {
                return (100);

            } else if (RightCurrVeloc < sspeed - 100) {
                return (sspeed + 275);

            } else {
                return (sspeed - 375);
            }

        } else if (Objects.equals(shoottype, "BLUEFAR")) {
            if (RightCurrVeloc > sspeed + 100) {
                return (100);

            } else if (RightCurrVeloc < sspeed - 100) {
                return (sspeed + 275);

            } else {
                return (sspeed - 475);
            }


        }
        return (0);
    }
    public double outtakeleft(double sspeedx, double sspeed, String shoottype, double LeftCurrVeloc) {
        BlueCloseAuto farauto = new BlueCloseAuto();

        if (Objects.equals(shoottype, "CLOSE")) {
            if (LeftCurrVeloc > sspeed + 25) {
                return (0);
            } else if (LeftCurrVeloc < sspeed - 25) {
                return (sspeed + (sspeed * sspeedx));
            } else {

                return (sspeed);

            }
        }
        else if (Objects.equals(shoottype, "REDFAR")){
            if (LeftCurrVeloc > sspeed + 100) {
                return (100);

            } else if (LeftCurrVeloc < sspeed - 100) {
                return (sspeed + 275);

            } else {
                return (sspeed - 375);

            }
        } else if (Objects.equals(shoottype, "BLUEFAR")) {
            if (LeftCurrVeloc > sspeed + 100) {
                return (100);

            } else if (LeftCurrVeloc < sspeed - 100) {
                return (sspeed + 275);

            } else {
                return (sspeed - 475);
            }
        }
        return (0);
    }
}
