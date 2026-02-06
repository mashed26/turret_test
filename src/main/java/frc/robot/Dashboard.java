// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.teamscreamrobotics.dashboard.DashboardBoolean;

/** Add your docs here. */
public class Dashboard {

  // Tabs to make within the dashboard.
  private static String overrides = "Overrides";
  private static String vision = "Vision";

  // Variables that can be adjusted within the dashboard, these are the variables that are used
  // within the code.
  public static DashboardBoolean disableAmbiguityRejection;
  // public static DashboardBoolean disableCoralRequirement;
  public static DashboardBoolean disableAllVisionUpdates;
  public static DashboardBoolean useGlobalEstimateForAutoAlign;
  public static DashboardBoolean ferryMode;

  // public static DashboardNumber autoScoreDistance;

  static {
    initialize();
  }

  public static void initialize() {
    // These are the variables that are created within the dashboard.
    // disableCoralRequirement = new DashboardBoolean(overrides, "Disable Coral Requirement",
    // false);
    disableAmbiguityRejection = new DashboardBoolean(vision, "Disable Ambiguity Rejection", false);
    disableAllVisionUpdates = new DashboardBoolean(vision, "Disable All Vision Updates", false);
    useGlobalEstimateForAutoAlign =
        new DashboardBoolean(vision, "Use Global Estimate For Auto Align", false);
    ferryMode = new DashboardBoolean(overrides, "Ferry Mode", false);

    // autoScoreDistance = new DashboardNumber(overrides, "Auto Score Distance", 0.03);
  }
}
