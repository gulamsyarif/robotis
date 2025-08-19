#!/usr/bin/env python3

import numpy as np
from zmp_com_support import calculate_com, calculate_zmp, calculate_support_polygon
from shapely.geometry import Polygon, Point

# Function to check if the robot is balanced
def is_robot_balanced(zmp_x, zmp_y, support_polygon_vertices):
    """
    Check if the robot is balanced by determining if the ZMP is within the support polygon.

    :param zmp_x: X coordinate of the Zero Moment Point (ZMP)
    :param zmp_y: Y coordinate of the Zero Moment Point (ZMP)
    :param support_polygon_vertices: List of tuples representing the (x, y) vertices of the support polygon
    :return: True if the robot is balanced, False otherwise
    """
    # Create a Polygon from the support polygon vertices
    support_polygon = Polygon(support_polygon_vertices)

    # Create a Point for the ZMP location
    zmp_point = Point(zmp_x, zmp_y)

    # Check if the ZMP is within the support polygon
    return support_polygon.contains(zmp_point)

# Function to analyze balance based on the ZMP and CoM
def analyze_balance(com_x, com_y, zmp_x, zmp_y, foot_positions):
    """
    Perform a full balance analysis based on the CoM, ZMP, and support polygon.

    :param com_x: X coordinate of the Center of Mass (CoM)
    :param com_y: Y coordinate of the Center of Mass (CoM)
    :param zmp_x: X coordinate of the Zero Moment Point (ZMP)
    :param zmp_y: Y coordinate of the Zero Moment Point (ZMP)
    :param foot_positions: List of tuples representing the (x, y) positions of the feet
    :return: A dictionary with balance evaluation results
    """
    # Construct the support polygon from foot positions
    support_polygon_vertices = foot_positions

    # Calculate if the robot is balanced based on the ZMP
    balanced = is_robot_balanced(zmp_x, zmp_y, support_polygon_vertices)

    # Create an analysis result dictionary
    analysis_result = {
        'CoM': (com_x, com_y),
        'ZMP': (zmp_x, zmp_y),
        'Support Polygon': support_polygon_vertices,
        'Balanced': balanced
    }

    return analysis_result

# Example usage of the balance analysis
if __name__ == '__main__':
    # Get real foot positions (example values or could be from actual sensors)
    foot_positions = calculate_support_polygon()  # Assume this returns the foot positions

    # Get real CoM values from IMU and relevant calculations
    com_x, com_y = calculate_com()  # Based on sensor readings

    # Get real ZMP values from IMU or based on dynamics equations
    zmp_x, zmp_y = calculate_zmp()  # Based on sensor readings and ZMP calculation

    # Perform the balance analysis using actual calculated values
    balance_results = analyze_balance(com_x, com_y, zmp_x, zmp_y, foot_positions)

    # Output the balance results
    print(f"Balance Analysis Result: {balance_results}")
    print(f"Is the robot balanced? {balance_results['Balanced']}")
