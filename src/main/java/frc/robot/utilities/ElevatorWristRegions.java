// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.Optional;

import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;

public class ElevatorWristRegions {

    public enum RegionType {
        CORAL_ONLY,                  //  Region when only holding coral
        STANDARD                     //  Region for all other situations
    }

    // All of the information about a region
    public class Region {
        public final RegionType type;                           // Type for this region
        public final int regionIndex;                           // Region numbering FOR THIS RegionType, from 0 (bottom) to highest (top)
        public final double elevatorMin, elevatorMax;           // Elevator min and max heights for this region, in inches
        public final double wristMin, wristMax;                 // Wrist min and max angles for this region, in degrees
        private Optional<Region> regionAbove, regionBelow;      // Region above (or empty if this is the top) or below (or empty if this is the bottom)

        /**
         * Creates a region.  TODO fill out JavaDocs
         * @param type          Type for this region
         * @param regionIndex   Region numbering FOR THIS RegionType, from 0 (bottom) to highest (top)
         * @param elevatorMin   Elevator min height for this region, in inches
         * @param elevatorMax   Elevator max height for this region, in inches
         * @param wristMin      Wrist min angle for this region, in degrees
         * @param wristMax      Wrist max angle for this region, in degrees
         */
        public Region(RegionType type, int regionIndex, double elevatorMin, double elevatorMax,
                      double wristMin, double wristMax) {
            this.type = type;
            this.regionIndex = regionIndex;
            this.elevatorMax = elevatorMax;
            this.elevatorMin = elevatorMin;
            this.wristMax = wristMax;
            this.wristMin = wristMin;
            regionAbove = Optional.empty();
            regionAbove = Optional.empty();
        }

        /**
         * Sets the regions above/below this region
         * @param below Region below, or null if none
         * @param above Region above, or null if none
         */
        public void setNeighborRegions(Region below, Region above) {
            regionBelow = (below==null) ? Optional.empty() : Optional.of(below);
            regionAbove = (above==null) ? Optional.empty() : Optional.of(above);
        }

        /**
         * Gets the region immediately above this region, or Optional.empty() if this is the top region
         * @return
         */
        public Optional<Region> getRegionAbove() {
            return regionAbove;
        }

        /**
         * Gets the region immediately below this region, or Optional.empty() if this is the bottom region
         * @return
         */
        public Optional<Region> getRegionBelow() {
            return regionBelow;
        }
    }

    // Reference to necessary subsystems.  Note that they may be null!!!!
    Elevator elevator;
    CoralEffector coralEffector;
    AlgaeGrabber algaeGrabber;

    // Arrays for the regions
    private final Region[] CoralOnlyRegions = {
        new Region(RegionType.CORAL_ONLY, 0, 0, 10, 69.5, 90),
        new Region(RegionType.CORAL_ONLY, 1, 10, 50, 50, 90),
        new Region(RegionType.CORAL_ONLY, 2, 50, 67, 82, 90),
        new Region(RegionType.CORAL_ONLY, 3, 67, 72, 20, 90),
        new Region(RegionType.CORAL_ONLY, 4, 72, 82, 27, 90)
    };
    private final Region[] StandardRegions = {
        new Region(RegionType.STANDARD, 0, 0, 5, 69.5, 90),
        new Region(RegionType.STANDARD, 1, 5, 23, 50, 90),
        new Region(RegionType.STANDARD, 2, 23, 82, 82, 90)
    };

    /**
     * Creates the ElevatorWristRegions system.
     * <p><b> Be sure </b> to register the subsystems to this object in RobotContainer constructor!!!!
     */
    public ElevatorWristRegions() {
        // Set the links to each region above/below
        for (Region region : CoralOnlyRegions) {
            region.setNeighborRegions(
                region.regionIndex <= 0 ? null : CoralOnlyRegions[region.regionIndex-1],
                region.regionIndex >= CoralOnlyRegions.length-1 ? null : CoralOnlyRegions[region.regionIndex+1]
            );
        }
        for (Region region : StandardRegions) {
            region.setNeighborRegions(
                region.regionIndex <= 0 ? null : StandardRegions[region.regionIndex-1],
                region.regionIndex >= StandardRegions.length-1 ? null : StandardRegions[region.regionIndex+1]
            );
        }
    }

    /**
     * Provide a link to the necessary subsystems
     * @param elevator
     * @param coralEffector
     * @param algaeGrabber
     */
    public void registerSubsystems(Elevator elevator, CoralEffector coralEffector, AlgaeGrabber algaeGrabber) {
        this.elevator = elevator;
        this.coralEffector = coralEffector;
        this.algaeGrabber = algaeGrabber;
    }

    /**
     * Returns the current region type, based on piece presence in the grabbers/effectors.
     * @return RegionType = CORAL_ONLY or STANDARD
     */
    public RegionType getCurrentRegionType() {
        return (coralEffector.isCoralPresent() && !algaeGrabber.isAlgaePresent()) ? RegionType.CORAL_ONLY : RegionType.STANDARD;
    }

    /**
     * Returns the current region type, based on piece presence in the grabbers/effectors and the elevator height
     * @return Region
     */
    public Region getCurrentRegion() {
        Region currentRegion;

        currentRegion = CoralOn
        for (int i = 0; i < CoralOnlyRegions.length; i++) {
            
        }


        return currentRegion;
    }
}
