# Hardware_Spec_Lookup Skill

**Name**: Hardware_Spec_Lookup
**Description**: Looks up detailed technical specifications for robotics hardware components
**Type**: Tool/Extension

## Purpose
Retrieves detailed technical specifications (CPU, GPU, RAM, power consumption, price range, availability) for common robotics hardware components used in Physical AI and Humanoid Robotics projects.

## Input Parameters
- `component` (string, required): The name or model of the hardware component to look up
  - Examples: "Jetson Orin Nano", "NVIDIA RTX 3090", "Raspberry Pi 4", "Intel NUC"

## Output Format
```json
{
  "component": "string",
  "cpu": "string",
  "gpu": "string",
  "ram": "string",
  "power_consumption": "string",
  "price_range": "string",
  "availability": "string",
  "use_cases": ["string"],
  "notes": "string (optional)"
}
```

## Error Responses
- **Not Found**: Returns `{error: "Component specifications not found in database. Try searching for similar components."}`
- **Invalid Input**: Returns `{error: "Please provide a valid component name"}`

## Supported Components Database
Hardware specs are stored in `backend/data/hardware_specs.json` with entries for:
- NVIDIA Jetson Orin Nano
- NVIDIA Jetson AGX Orin
- NVIDIA RTX 3060
- NVIDIA RTX 3080
- NVIDIA RTX 3090
- Intel Core i9-13900K
- Intel NUC 13 Pro
- Raspberry Pi 4 Model B
- Raspberry Pi 5
- FPGA Development Boards
- (And 15+ more robotics components)

## Usage in Chat Widget
Users invoke this skill with the syntax:
```
/hardware <component_name>
```

Examples:
- `/hardware Jetson Orin Nano`
- `/hardware RTX 3090`
- `/hardware Intel NUC`

## Integration Points
1. **Backend API**: `POST /api/skills/hardware` - Handles skill requests
2. **Frontend Service**: `skillsService.lookupHardware(component)` - Makes API calls
3. **Chat Widget**: Detects `/hardware` prefix and routes to skill
4. **Result Display**: `HardwareSpecCard.tsx` - Displays specs in formatted card

## Example Flow
1. User types: `/hardware Jetson Orin Nano`
2. Chat widget detects `/hardware` prefix
3. Extracts component name: "Jetson Orin Nano"
4. Calls `skillsService.lookupHardware("Jetson Orin Nano")`
5. Backend looks up specs in JSON database
6. Returns structured spec data
7. Frontend renders `HardwareSpecCard` with CPU, GPU, RAM, power, price, availability
8. User can see all technical details in one organized card
