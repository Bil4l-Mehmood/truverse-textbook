"""Skills API routes for Hardware and ROS2 tools."""

import logging
from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field
from src.services.skills.hardware_lookup import lookup_hardware
from src.services.skills.ros2_generator import generate_ros2_command

logger = logging.getLogger(__name__)
router = APIRouter()


class HardwareRequest(BaseModel):
    """Request model for hardware lookup."""
    component: str = Field(..., description="Hardware component name to look up")


class HardwareResponse(BaseModel):
    """Response model for hardware specs."""
    component: str
    cpu: str
    gpu: str
    ram: str
    power_consumption: str
    price_range: str
    availability: str
    use_cases: list


class ROS2Request(BaseModel):
    """Request model for ROS2 command generation."""
    task: str = Field(..., description="ROS 2 task description")


class ROS2Response(BaseModel):
    """Response model for ROS2 commands."""
    task: str
    command: str
    explanation: str
    parameters: list


@router.post("/hardware", response_model=dict)
async def lookup_hardware_endpoint(request: HardwareRequest):
    """
    Look up hardware specifications by component name.
    
    Example: {"component": "Jetson Orin Nano"}
    """
    try:
        result = lookup_hardware(request.component)
        logger.info(f"Hardware lookup for: {request.component}")
        return result
    except Exception as e:
        logger.error(f"Hardware lookup error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to look up hardware specifications"
        )


@router.post("/ros2", response_model=dict)
async def generate_ros2_endpoint(request: ROS2Request):
    """
    Generate ROS 2 commands based on task description.
    
    Example: {"task": "launch a LiDAR sensor node"}
    """
    try:
        result = generate_ros2_command(request.task)
        logger.info(f"ROS2 command generated for: {request.task}")
        return result
    except Exception as e:
        logger.error(f"ROS2 generation error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to generate ROS 2 command"
        )
