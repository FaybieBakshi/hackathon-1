# Docusaurus Configuration Contract

## API: docusaurus.config.js

### Purpose
Defines the configuration for the Docusaurus documentation site, including site metadata, plugins, themes, and customizations.

### Interface Definition

```javascript
module.exports = {
  // Site metadata
  title: string,           // Site title
  tagline: string,         // Site tagline/description
  url: string,            // URL of the project
  baseUrl: string,        // Base URL for the site
  onBrokenLinks: string,  // Behavior when encountering broken links
  onBrokenMarkdownLinks: string, // Behavior for broken markdown links
  favicon: string,        // Path to favicon

  // Organization metadata
  organizationName: string, // GitHub username/organization
  projectName: string,     // GitHub repository name
  deploymentBranch: string, // Branch for deployment

  // Themes and styling
  presets: array,         // Docusaurus presets
  themes: array,          // Additional themes
  plugins: array,         // Docusaurus plugins
  themeConfig: object,    // Theme-specific configuration

  // Custom configurations
  customFields: object    // Additional custom fields
}
```

### Required Fields
- `title`: Site title (string)
- `tagline`: Site description (string)
- `url`: Production URL of the site (string)
- `baseUrl`: Base URL for the site (string)
- `favicon`: Path to favicon (string)
- `organizationName`: GitHub organization name (string)
- `projectName`: GitHub repository name (string)
- `presets`: Array of Docusaurus presets (array)

### Validation Rules
1. `url` must be a valid URL with protocol (https://)
2. `baseUrl` must start with "/" and end with "/"
3. `onBrokenLinks` must be one of: "ignore", "log", "throw"
4. `onBrokenMarkdownLinks` must be one of: "ignore", "log", "throw"
5. `presets` must include at least one valid Docusaurus preset

### Default Values
- `onBrokenLinks`: "throw"
- `onBrokenMarkdownLinks`: "warn"
- `deploymentBranch`: "gh-pages"

## API: sidebars.js

### Purpose
Defines the navigation structure for the documentation site, organizing content into hierarchical categories.

### Interface Definition

```javascript
module.exports = {
  docs: [
    {
      type: string,       // "category" or "doc"
      label: string,      // Display name for category
      items: array,       // Array of child items
      collapsed: boolean, // Whether category is collapsed by default
      link: object        // Optional custom link configuration
    }
  ]
}
```

### Validation Rules
1. Each item in `docs` must have a valid `type` ("category" or "doc")
2. "category" items must have a `label` and `items` array
3. "doc" items must reference an existing document ID
4. Document IDs must correspond to actual markdown files

## API: Markdown Frontmatter

### Purpose
Defines metadata for individual documentation pages.

### Interface Definition

```yaml
---
title: string           # Page title
description: string     # Page description for SEO
slug: string           # Custom URL slug (optional)
sidebar_label: string  # Label to show in sidebar (optional)
tags: [string]         # Array of tags for the document
---
```

### Required Fields
- `title`: Document title (string) - when different from filename

### Validation Rules
1. All string values must be non-empty
2. `slug` must be URL-safe (alphanumeric, hyphens, underscores only)
3. `tags` array elements must be non-empty strings