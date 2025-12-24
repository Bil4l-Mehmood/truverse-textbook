/**
 * Skills service for interacting with Claude Code Skills API
 */

const API_BASE_URL = typeof window !== 'undefined' && window.location.hostname === 'localhost'
  ? 'http://localhost:8000'
  : `${window.location.protocol}//${window.location.host}`;

export interface HardwareSpec {
  component: string;
  cpu: string;
  gpu: string;
  ram: string;
  power_consumption: string;
  price_range: string;
  availability: string;
  use_cases: string[];
  error?: string;
  suggestion?: string;
  note?: string;
}

export interface ROS2Command {
  task: string;
  command: string;
  explanation: string;
  parameters: Array<{
    name: string;
    description: string;
    default?: string;
  }>;
  clarification_needed?: string;
  suggestions?: string[];
  error?: string;
}

/**
 * Look up hardware specifications
 */
export async function lookupHardware(component: string): Promise<HardwareSpec> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/v1/skills/hardware`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ component }),
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.detail || 'Failed to lookup hardware');
    }

    return await response.json();
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Hardware lookup failed';
    console.error('[SkillsService] Hardware lookup error:', message);
    throw new Error(message);
  }
}

/**
 * Generate ROS 2 commands
 */
export async function generateROS2Command(task: string): Promise<ROS2Command> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/v1/skills/ros2`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ task }),
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.detail || 'Failed to generate ROS2 command');
    }

    return await response.json();
  } catch (error) {
    const message = error instanceof Error ? error.message : 'ROS2 command generation failed';
    console.error('[SkillsService] ROS2 generation error:', message);
    throw new Error(message);
  }
}
