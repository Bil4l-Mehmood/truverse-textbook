"""Hardware specification lookup service."""

import json
import os
from difflib import SequenceMatcher
from typing import Optional, Dict, Any

# Load hardware specs from JSON file
HARDWARE_SPECS_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))),
    "data",
    "hardware_specs.json"
)


def load_hardware_specs() -> list:
    """Load hardware specs from JSON file."""
    try:
        with open(HARDWARE_SPECS_PATH, 'r') as f:
            data = json.load(f)
            return data.get('components', [])
    except (FileNotFoundError, json.JSONDecodeError):
        return []


def _fuzzy_match(query: str, target: str, threshold: float = 0.6) -> bool:
    """Check if query fuzzy matches target string."""
    ratio = SequenceMatcher(None, query.lower(), target.lower()).ratio()
    return ratio >= threshold


def lookup_hardware(component_name: str) -> Dict[str, Any]:
    """
    Look up hardware specifications by component name.
    
    Args:
        component_name: Name of the hardware component (fuzzy matching supported)
    
    Returns:
        Dictionary with hardware specs or error message
    """
    if not component_name or not component_name.strip():
        return {
            "error": "Please provide a valid component name",
            "example_queries": [
                "Jetson Orin Nano",
                "RTX 3090",
                "Intel NUC",
                "Raspberry Pi 5"
            ]
        }
    
    component_name = component_name.strip()
    specs = load_hardware_specs()
    
    # Try exact match first
    for component in specs:
        if component['name'].lower() == component_name.lower():
            return {
                "component": component['name'],
                "cpu": component.get('cpu', 'N/A'),
                "gpu": component.get('gpu', 'N/A'),
                "ram": component.get('ram', 'N/A'),
                "power_consumption": component.get('power', 'N/A'),
                "price_range": component.get('price', 'N/A'),
                "availability": component.get('availability', 'N/A'),
                "use_cases": component.get('use_cases', [])
            }
    
    # Try fuzzy matching
    best_match = None
    best_ratio = 0
    
    for component in specs:
        ratio = SequenceMatcher(None, component_name.lower(), component['name'].lower()).ratio()
        if ratio > best_ratio:
            best_ratio = ratio
            best_match = component
    
    if best_match and best_ratio > 0.7:
        return {
            "component": best_match['name'],
            "cpu": best_match.get('cpu', 'N/A'),
            "gpu": best_match.get('gpu', 'N/A'),
            "ram": best_match.get('ram', 'N/A'),
            "power_consumption": best_match.get('power', 'N/A'),
            "price_range": best_match.get('price', 'N/A'),
            "availability": best_match.get('availability', 'N/A'),
            "use_cases": best_match.get('use_cases', []),
            "note": f"(Did you mean: {best_match['name']}?)"
        }
    
    # No match found
    return {
        "error": "Component specifications not found in database",
        "suggestion": "Try searching for: Jetson Orin Nano, RTX 3090, RTX 3080, Intel NUC, or Raspberry Pi"
    }
