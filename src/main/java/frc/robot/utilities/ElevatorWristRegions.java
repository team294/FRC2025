// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorWristRegions {

    public enum RegionType {
        CORAL_ONLY,                  //  Region when only holding coral
        STANDARD                     //  Region for all other situations
    }

    // All of the information about a region
    public static class Region {
        public final RegionType type;                           // Type for this region
        public final int regionIndex;                           // Region numbering FOR THIS RegionType, from 0 (bottom) to highest (top)
        public final double elevatorMin, elevatorMax;           // Elevator min and max heights for this region, in inches
        public final double wristMin, wristMax;                 // Wrist min and max angles for this region, in degrees
        private Optional<Region> regionAbove, regionBelow;      // Region above (or empty if this is the top) or below (or empty if this is the bottom)

        /**
         * Creates a region for a region type and a range of elevator heights
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

    // Arrays for the regions
    // Region indicies must be in order from 0, 1, 2, etc
    // Note that some regions have added 1-2" of margin on the elevator position for safety
    public static final Region[] CoralOnlyRegions = {
        new Region(RegionType.CORAL_ONLY, 0, ElevatorConstants.ElevatorPosition.LOWER_LIMIT.value, 4, 69.5, 97),
        new Region(RegionType.CORAL_ONLY, 1, 4, 25, 70, 80),
        new Region(RegionType.CORAL_ONLY, 2, 25, 30, 70, 100),
        new Region(RegionType.CORAL_ONLY, 3, 30, ElevatorConstants.ElevatorPosition.UPPER_LIMIT.value, 96.5, 100.5)
    };
    private static final Region[] StandardRegions = {
        new Region(RegionType.STANDARD, 0, ElevatorConstants.ElevatorPosition.LOWER_LIMIT.value, 5, 69.5, 97),
        new Region(RegionType.STANDARD, 1, 5, 25, -21, 80),
        new Region(RegionType.STANDARD, 2, 25, ElevatorConstants.ElevatorPosition.UPPER_LIMIT.value, -30, 97)
    };

    /**
     * Link the prior and next regions in each array
     */
    static {
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
     * Returns a region, based on the input region type and elevatorHeight
     * @param type  CORAL_ONLY or STANDARD
     * @param elevatorHeight
     * @return Region
     */
    public static Region GetRegion(RegionType type, double elevatorHeight) {
        Region currentRegion;

        // Clamp the elevator height to be within the region arrays
        elevatorHeight = MathUtil.clamp(elevatorHeight, 
            ElevatorConstants.ElevatorPosition.LOWER_LIMIT.value, 
            ElevatorConstants.ElevatorPosition.UPPER_LIMIT.value - 0.001);

        if (type == RegionType.CORAL_ONLY) {
            currentRegion = CoralOnlyRegions[0];  // default value
            for (Region r : CoralOnlyRegions) {
                if ( elevatorHeight >= r.elevatorMin && elevatorHeight < r.elevatorMax) {
                    currentRegion = r;
                    break;
                }
            }
        } else {
            currentRegion = StandardRegions[0];  // default value
            for (Region r : StandardRegions) {
                if ( elevatorHeight >= r.elevatorMin && elevatorHeight < r.elevatorMax) {
                    currentRegion = r;
                    break;
                }
            }
        }

        return currentRegion;
    }

}
