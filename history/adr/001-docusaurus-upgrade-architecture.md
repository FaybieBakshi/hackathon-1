# ADR: Docusaurus Upgrade Architecture

## Status
Proposed

## Context
The frontend-ai-book Docusaurus project requires an upgrade to modernize its content structure and UI. The current implementation has a flat content organization that doesn't scale well for multi-module navigation. We need to make key architectural decisions about the technology stack, content organization, and styling approach.

## Decision

### 1. Theme Selection: @docusaurus/theme-classic with Custom CSS
- **Decision**: Use the default @docusaurus/theme-classic with custom CSS modifications
- **Rationale**: Avoids over-engineering while maintaining modern look and feel; reduces complexity compared to custom theme development
- **Alternatives considered**:
  - Custom theme from scratch (higher complexity)
  - Third-party themes (less control, potential maintenance issues)

### 2. Content Structure: Hierarchical Organization
- **Decision**: Organize content in a hierarchical sidebar structure with modules and chapters
- **Rationale**: Better for multi-module navigation and improved user experience
- **Alternatives considered**:
  - Flat structure (becomes unwieldy with many modules)
  - Category-based organization (less intuitive for educational content)

### 3. Styling Approach: Custom CSS Files
- **Decision**: Use custom CSS via dedicated files (custom.css, components.css, design-system.css)
- **Rationale**: Minimal bundle size, direct control over styling, simpler than CSS-in-JS or frameworks
- **Alternatives considered**:
  - CSS Modules (more complex setup)
  - Tailwind CSS (larger bundle size)
  - Styled Components (overkill for documentation site)

### 4. Asset Management: Static Folder
- **Decision**: Use the /static folder for assets
- **Rationale**: Simple approach that works well with Docusaurus, no build complications
- **Alternatives considered**:
  - Importing assets in components (unnecessary complexity for static docs)
  - CDN hosting (unnecessary for this use case)

## Consequences

### Positive
- Maintainable and scalable content structure
- Familiar Docusaurus theme with custom enhancements
- Minimal bundle size with direct CSS control
- Simple asset management approach
- Consistent with Docusaurus best practices

### Negative
- Custom CSS requires more manual maintenance than utility frameworks
- Hierarchical structure requires careful navigation design
- Limited styling flexibility compared to more advanced solutions

## Alternatives Considered
1. Complete theme rebuild: Would provide maximum customization but at high complexity cost
2. Third-party documentation themes: Would reduce development time but limit customization options
3. Flat content structure with tags: Would be simpler but less navigable for large content sets

## Assumptions
- The team has CSS expertise to maintain custom styles
- The content will continue to grow in a modular fashion
- Performance requirements can be met with the selected approach
- The RAG system integration will remain compatible with the new structure