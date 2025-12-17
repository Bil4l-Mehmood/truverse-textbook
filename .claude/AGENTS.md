# Claude Code Subagents Guide

## Overview

The AI Textbook Platform integrates Claude Code Subagents to automate and enhance various educational tasks. Specialized agents can analyze content, review code, provide tutoring, conduct research, and debug errors.

## Available Agents

### 1. Content Expert Agent
Analyzes educational material and extracts learning objectives.

**Capabilities:**
- Extract learning objectives from chapter content
- Identify key concepts and main topics
- Assess content difficulty level
- Generate study guides and concept maps
- Create prerequisites and learning paths

**Usage:**
```python
from .agent import ContentExpertAgent

agent = ContentExpertAgent()
result = await agent.analyze_chapter(
    content="Chapter on ROS 2...",
    chapter_name="Week 1: ROS 2 Foundations"
)
```

### 2. Code Reviewer Agent
Reviews and provides feedback on code quality and correctness.

**Capabilities:**
- Analyze code correctness
- Assess code style and quality
- Suggest performance optimizations
- Conduct security review
- Generate test cases

**Usage:**
```python
from .agent import CodeReviewerAgent

agent = CodeReviewerAgent()
result = await agent.review_solution(
    code="student_code_here",
    assignment_id="assignment_001"
)
```

### 3. Tutor Agent
Provides educational explanations and tutoring at appropriate difficulty levels.

**Capabilities:**
- Explain concepts clearly
- Generate relevant examples
- Provide interactive Q&A
- Create assessments
- Adapt explanations to skill level

**Usage:**
```python
from .agent import TutorAgent

agent = TutorAgent()
result = await agent.explain_concept(
    concept="Publisher-Subscriber Pattern",
    difficulty_level="intermediate"
)
```

### 4. Researcher Agent
Analyzes research papers and identifies trends.

**Capabilities:**
- Summarize research papers
- Extract key findings
- Analyze citations
- Identify related work
- Suggest future research directions

**Usage:**
```python
from .agent import ResearcherAgent

agent = ResearcherAgent()
result = await agent.analyze_paper(
    paper_title="Learning Dexterous In-Hand Manipulation",
    topic="humanoid_dexterity"
)
```

### 5. Debugger Agent
Helps identify and fix errors in code.

**Capabilities:**
- Root cause analysis
- Error pattern recognition
- Fix suggestions
- Prevention strategies
- Test case generation

**Usage:**
```python
from .agent import DebuggerAgent

agent = DebuggerAgent()
result = await agent.debug_error(
    error_message="AttributeError: 'Node' has no attribute 'publisher'",
    code_context="ros2_node.py code snippet..."
)
```

## Agent Orchestrator

For complex workflows requiring multiple agents:

```python
from .agent import AgentOrchestrator

orchestrator = AgentOrchestrator()

# Single agent tasks
await orchestrator.analyze_content(chapter_content, chapter_name)
await orchestrator.review_code(student_code, assignment_id)
await orchestrator.explain_concept(concept, difficulty_level)

# Multi-agent workflows
result = await orchestrator.comprehensive_learning_plan("Humanoid Robotics")
result = await orchestrator.research_and_explain("Reinforcement Learning in Robotics")
```

## Multi-Agent Workflows

### Comprehensive Learning Plan
Orchestrates Content Expert, Tutor, and Researcher agents to create complete learning paths.

**Process:**
1. Content Expert: Analyzes existing materials
2. Researcher: Finds latest research and trends
3. Tutor: Creates learning sequence and examples
4. Output: Learning objectives, materials, assessments

**Example:**
```python
result = await orchestrator.comprehensive_learning_plan(
    topic="Motion Planning in Robotics"
)
```

### Research and Explain
Combines Researcher and Tutor agents for in-depth topic understanding.

**Process:**
1. Researcher: Analyzes papers and trends
2. Tutor: Prepares clear explanations
3. Output: Summary, examples, resources

**Example:**
```python
result = await orchestrator.research_and_explain(
    research_topic="Vision Transformers for Robotics"
)
```

### Code Review Workflow
Code Reviewer and Debugger agents work together for comprehensive code assessment.

**Process:**
1. Code Reviewer: Initial quality analysis
2. Debugger (if needed): Fix identified issues
3. Output: Feedback, improvements, tests

## Integration with Backend

### Via FastAPI Endpoints

```python
from fastapi import FastAPI
from .agent import AgentOrchestrator

app = FastAPI()
orchestrator = AgentOrchestrator()

@app.post("/api/v1/agents/analyze-content")
async def analyze_content(request: ContentAnalysisRequest):
    result = await orchestrator.analyze_content(
        request.content,
        request.chapter_name
    )
    return result

@app.post("/api/v1/agents/review-code")
async def review_code(request: CodeReviewRequest):
    result = await orchestrator.review_code(
        request.code,
        request.assignment_id
    )
    return result
```

### Via Frontend Components

```typescript
// React component using agent API
import { useState } from 'react';

export function CodeReviewPanel() {
  const [code, setCode] = useState('');
  const [feedback, setFeedback] = useState(null);

  const reviewCode = async () => {
    const response = await fetch('/api/v1/agents/review-code', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        code,
        assignment_id: 'assignment_001'
      })
    });
    const result = await response.json();
    setFeedback(result.result);
  };

  return (
    <div>
      <textarea value={code} onChange={(e) => setCode(e.target.value)} />
      <button onClick={reviewCode}>Get Code Review</button>
      {feedback && <div>{JSON.stringify(feedback, null, 2)}</div>}
    </div>
  );
}
```

## Best Practices

### 1. Context Matters
Provide detailed context for better agent performance:
```python
# Good - specific context
await agent.analyze_chapter(
    content=chapter_content,
    chapter_name="Week 3: Computer Vision Basics"
)

# Better - add prerequisites
# Include metadata about student level, prerequisites, etc.
```

### 2. Error Handling
Always handle agent execution errors:
```python
try:
    result = await agent.analyze_chapter(content, chapter_name)
except AgentExecutionError as e:
    logger.error(f"Agent failed: {e}")
    # Fallback to manual processing
```

### 3. Caching Results
Cache agent outputs for expensive operations:
```python
@cache(ttl=3600)  # Cache for 1 hour
async def get_chapter_analysis(chapter_id: str):
    result = await orchestrator.analyze_content(
        chapter_content,
        f"Chapter {chapter_id}"
    )
    return result
```

### 4. Progressive Complexity
Start with simple agents, orchestrate for complex workflows:
```python
# Level 1: Simple analysis
analysis = await orchestrator.analyze_content(content, name)

# Level 2: Multi-agent workflow
learning_plan = await orchestrator.comprehensive_learning_plan(topic)

# Level 3: Custom orchestration
custom_workflow = [
    await orchestrator.analyze_research(paper1, topic),
    await orchestrator.analyze_research(paper2, topic),
    await orchestrator.explain_concept(combined_topic, "advanced")
]
```

## Monitoring and Logging

### Agent Execution Tracking
```python
import logging

logger = logging.getLogger(__name__)

async def tracked_execution():
    logger.info(f"[Agent] Starting {agent.role.value} for task: {agent.task}")
    try:
        result = await agent.execute()
        logger.info(f"[Agent] Completed successfully")
        return result
    except Exception as e:
        logger.error(f"[Agent] Failed: {e}")
        raise
```

### Metrics
- Agent execution time
- Success/failure rates
- Cache hit rates
- User satisfaction with agent output

## Performance Optimization

### Parallel Agent Execution
```python
import asyncio

results = await asyncio.gather(
    orchestrator.analyze_content(content1, name1),
    orchestrator.analyze_content(content2, name2),
    orchestrator.analyze_content(content3, name3),
)
```

### Lazy Loading
```python
from functools import lru_cache

@lru_cache(maxsize=10)
async def get_agent(agent_type: str):
    # Initialize and cache agent
    return agents[agent_type]
```

## Future Enhancements

- [ ] Custom agent types for specific domains
- [ ] Agent training on platform content
- [ ] Real-time feedback during execution
- [ ] Integration with external AI services
- [ ] Agent collaboration patterns
- [ ] Advanced orchestration workflows

## Troubleshooting

### Agent Not Responding
1. Check agent configuration
2. Verify Claude Code SDK is installed
3. Check system resources
4. Review logs for errors

### Poor Agent Output
1. Provide more detailed context
2. Adjust difficulty level
3. Try different agent type
4. Check agent system prompt

### Performance Issues
1. Enable caching
2. Use parallel execution
3. Reduce context size
4. Check resource utilization

## Examples

See `.claude/agent.py` for complete working examples of all agent types.

## Support

For issues or questions about agents:
1. Check this documentation
2. Review example code in `agent.py`
3. Check system logs
4. Create an issue with detailed context
