// environmentData.cs
// This file defines the following in the simulation environment:
// 1. Course dimensions
// 2. LZ / OZ dimensions
// 3. Obstacle / LZ / OZ Materials

// All units here are in meters

// Obstacle specification dimensions can be found in ???
// LZ / OZ physics properties can be found in ???


using System;
using UnityEngine;

public static class environmentData
{ 

    // Mission 1.1: Productivity
    public const float depotwideWidth = 9f;
    public const float depotnarrowWidth = 1.5f;
    public const float depotDepth = 30f;
    public const float course1Width = 23f; // Picked an arbitrary value. Matches course width for Mission 1.3
    public const float course1Depth = 1609f; // Approx. 1 mile
    public const float endWallWidth = 23f; //TODO: Ask GoAero people how the Mission 1.1 course is limited. This assumes there's a wall at the end
    public const float endWallHeight = 7.5f; // Picked an arbitray value. Approx. 2 storeys


    // Mission 1.2: Adversity
    public const float baseWidth = 7.6f;
    public const float baseDepth = 15f;
    public const float pitLength = 3.7f;
    // public const float pitWallLength = ???;
    // TODO: Figure out the dimensions of walls, spacing bet. pit and walls
    public const float hillLength = 3.4f;
    public const float hillAngle = 12f; // in degrees
    public const float floodRadius = 3.6576f; // Approx 24ft diameter / 12ft radius
    public const float floodDepth = 0.5f;
    public const float tornadoLength = 4.6f;
    public const float unknownWidth = 18f;
    public const float unknownDepth = 7.6f;


    // Mission 1.3: Maneuvering
    public const float course3Width = 23f;
    public const float course3Depth = 69f; // course depth varies between 69m and 99m
    public const float spotWidth = 2.4f;
    public const float spotDepth = 2.4f;
    public const float obstacleDepthSpacing = course3Depth / 4; // Assuming obstacles 1 to 4 are evenly spaced, and obstacle 1 starts at the far end of the course
    public const float pylonDiameter = 2f;
    public const float pylonHeight = 10f;
    public const float wallHeight = 15f;
    public const float wallWidth = 10f; // TODO: Ask GoAero people for wall width and depth
    public const float wallDepth = 2f;

}
