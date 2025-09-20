# Contributing to LifeTrac

Thank you for your interest in contributing to the LifeTrac project! This document provides guidelines for contributing CAD designs, documentation, and improvements.

## Types of Contributions

### 1. CAD Design Improvements
- Enhanced existing designs
- New version variants
- Component optimizations
- Manufacturing improvements

### 2. Documentation
- Assembly instructions
- User manuals
- Technical specifications
- Safety guidelines

### 3. Experimental Designs
- Research prototypes
- New technology integration
- Concept designs
- Performance studies

## Contribution Process

### Before Contributing
1. Review existing versions to avoid duplication
2. Check the [issues](https://github.com/OpenSourceEcology/LifeTrac/issues) for planned work
3. Consider opening an issue to discuss major changes

### Making Contributions
1. **Fork the repository**
2. **Create a feature branch**: `git checkout -b feature/your-feature-name`
3. **Add your changes** following the directory structure
4. **Document your work** thoroughly
5. **Test your designs** when possible
6. **Submit a pull request**

## Directory Structure Guidelines

### For New Versions
```
versions/vX-description/
├── README.md              # Version-specific documentation
├── cad/                   # CAD files in multiple formats
├── documentation/         # Technical documentation
├── specifications/        # Detailed specifications
├── assembly/              # Assembly instructions (if applicable)
├── electronics/           # Electronic schematics (if applicable)
├── software/              # Control software (if applicable)
└── images/               # Photos, renderings, diagrams
```

### File Naming Conventions
- Use descriptive, lowercase names with hyphens
- Include version information in filenames
- Use standard file extensions (.step, .stl, .pdf, .md)

## CAD File Requirements

### Formats
- **Primary**: FreeCAD (.FCStd) for maximum compatibility and editability
- **Export**: STEP (.step, .stp) for CAD data exchange
- **Manufacturing**: STL (.stl) for 3D printing and visualization
- **2D Drawings**: PDF (.pdf) for documentation

### Quality Standards
- Clean, well-organized models
- Proper assemblies and constraints
- Accurate dimensions and tolerances
- Material specifications where applicable

## Documentation Standards

### README Files
Each version must include a comprehensive README.md with:
- Overview and features
- Specifications
- File structure explanation
- Assembly notes
- Safety considerations

### Technical Documentation
- Clear, concise language
- Proper technical terminology
- Referenced standards and specifications
- Safety warnings where appropriate

## Safety Considerations

All contributions must consider safety:
- Include appropriate safety warnings
- Follow relevant safety standards
- Consider operator and bystander safety
- Document known hazards and mitigation strategies

## Review Process

### Pull Request Requirements
- Clear description of changes
- Updated documentation
- Proper file organization
- Safety considerations addressed

### Review Criteria
- Technical accuracy
- Documentation completeness
- Safety considerations
- Compatibility with existing versions
- Manufacturing feasibility

## Community Guidelines

- Be respectful and constructive
- Help others learn and improve
- Share knowledge and experience
- Follow open-source principles
- Consider the global community

## Questions and Support

- **Issues**: Use GitHub Issues for questions and bug reports
- **Discussions**: Use GitHub Discussions for general questions
- **Wiki**: Check the [LifeTrac Wiki](https://wiki.opensourceecology.org/wiki/LifeTrac) for additional information

## License

By contributing to this project, you agree to license your contributions under the same terms as the project.