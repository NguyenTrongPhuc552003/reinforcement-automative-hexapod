# Hexapod Robot Documentation

This directory contains comprehensive documentation for the reinforcement learning-enabled hexapod robot project. The documentation covers hardware specifications, software architecture, quickstart guides, and detailed diagrams.

## Documentation Overview

| File                               | Description                                                                   |
|------------------------------------|-------------------------------------------------------------------------------|
| [architecture.md](architecture.md) | Detailed software architecture, design patterns, and component interactions   |
| [hardware.md](hardware.md)         | Hardware specifications, wiring diagrams, and component assembly instructions |
| [quickstart.md](quickstart.md)     | Getting started guide for new users to quickly set up and operate the hexapod |

## Diagram Documentation

The `diagrams/` directory contains PlantUML source files that generate various system diagrams:

| Diagram                                         | Description                                                |
|-------------------------------------------------|------------------------------------------------------------|
| [class.puml](diagrams/src/class.puml)           | Class diagrams showing software component relationships    |
| [component.puml](diagrams/src/component.puml)   | Component diagrams illustrating system modules             |
| [sequence.puml](diagrams/src/sequence.puml)     | Sequence diagrams for key interactions and processes       |
| [state.puml](diagrams/src/state.puml)           | State machine diagrams for robot control states            |
| [deployment.puml](diagrams/src/deployment.puml) | Deployment diagrams showing hardware/software distribution |
| [building.puml](diagrams/src/building.puml)     | Diagrams explaining the build and compilation process      |

## Generating Diagrams

The PlantUML diagrams can be generated into visual formats using the following commands:

```bash
# Install PlantUML (Debian/Ubuntu)
sudo apt install plantuml

# Generate all diagrams
./scripts/build.sh -l
```

Generated diagrams will be saved in the docs/diagrams/image/ directory as their source files with `.png` extensions.

## Viewing Documentation

For the best viewing experience, use a Markdown viewer or editor that supports:

- Tables
- Code highlighting
- Image embedding

You can view the documentation directly on GitHub or use tools like VSCode with the Markdown Preview extension.

## Documentation Structure

```t
docs/
├── README.md               # This file
├── architecture.md         # Software architecture details
├── hardware.md             # Hardware specifications and setup
├── quickstart.md           # Getting started guide
└── diagrams/               # System diagrams
    └── src/                # PlantUML source files
        ├── class.puml
        ├── component.puml
        ├── sequence.puml
        ├── state.puml
        ├── deployment.puml
        └── building.puml
```

## Contributing to Documentation
When contributing to the documentation, please follow these guidelines:
1. Use clear, concise language
2. Include code examples where appropriate
3. Add diagrams for complex concepts
4. Ensure code snippets are syntax-highlighted
5. Keep the documentation up to date with code changes

Document Formatting should follow these formatting conventions:
- Use headings (#, ##, ###) for section organization
- Use code blocks with language specifiers:
```cpp
// C++ code example
```
- Use tables for structured information
- Use bullet points for lists
- Include cross-references to other documents when relevant

## Related Resources
- [Project README](README.md): Main project overview
- [App Documentation](../app/README.md): Application-specific details
- [Driver Documentation](../driver/README.md): Low-level driver information
- [PyTD3 Documentation](../pytd3/README.md): Reinforcement learning framework
For additional help or to report documentation issues, please open an issue on the project's GitHub repository.