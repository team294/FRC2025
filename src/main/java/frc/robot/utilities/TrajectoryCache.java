// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.HashMap;
import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;


/**
 * Class that caches all trajectories that the robot could run.
 * Create one object instance of this class when the robot initializes to build the trajectories. 
 */

 public class TrajectoryCache {
    private FileLog log;
   
    public enum TrajectoryType {
        // Add Choreo trajectories here.  Syntax:
        //   TrajectoryConstantName("Trajectory file name"),
        TestPath1("TestPath1"),
        TestPath2("TestPath2"),
        TestPath3("TestPath3"),
        CirclePath("CirclePath"),
        Rotate180("Rotate180");


        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final String trajFileName;
        TrajectoryType(String trajFileName){ this.trajFileName = trajFileName; }
    }

    private static int trajectoryCount = TrajectoryType.values().length;
    private HashMap<TrajectoryType, Trajectory<SwerveSample>> cache = new HashMap<TrajectoryType,Trajectory<SwerveSample>>(trajectoryCount);    // HashMap of Choreo Trajectories

    /**
     * Load all trajectories when the robot starts up into a cache for trajectory-following commands.
     * @param log
     */
    public TrajectoryCache(FileLog log){
        this.log = log;
        for (TrajectoryType tt : TrajectoryType.values() ) {
            cacheTrajectory(tt);
        }
    }

    /**
     * Loads the choreo trajectory into the trajectory cache that has
     * the same name as the trajectoryType enum's trajFileName String
     * If a trajectory with that String is unable to be loaded a sticky fault
     * will be recorded and no trajectory will be added to the trajectory cache
     * @param trajectoryLoad the trajectoryType enum that will be loaded into 
     * the trajectory cache with the enum as the key
     */
    public void cacheTrajectory(TrajectoryType trajectoryLoad){
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory(trajectoryLoad.trajFileName);
        if(traj.isPresent()){
            cache.put(trajectoryLoad, traj.get());
        }
        else{
            RobotPreferences.recordStickyFaults("Trajectory-CacheTrajectory-" + trajectoryLoad.trajFileName +"-not-loaded", log);
        }
    }

    /**
     * Getter to get the choreo trajectory from the trajectory cache
     * @param key Key in the HashMap corresponding to the loaded trajectory
     * @return the Choreo trajectory in the trajectory cache.  Null if the requested trajectory
     * was not loaded successfully.
     */
    public Trajectory<SwerveSample> getTrajectory(TrajectoryType key){
        return cache.get(key);
    }
}


