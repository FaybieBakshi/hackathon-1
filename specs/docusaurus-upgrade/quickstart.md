# Quickstart: Docusaurus Project Upgrade

## Prerequisites

- Node.js (v18 or higher)
- npm or yarn package manager
- Git for version control
- Basic knowledge of Docusaurus and Markdown

## Setup Environment

1. **Clone the repository** (if not already done):
   ```bash
   git clone [repository-url]
   cd hackathon-1
   ```

2. **Navigate to the frontend directory**:
   ```bash
   cd frontend-ai-book
   ```

3. **Install dependencies**:
   ```bash
   npm install
   ```

## Development Workflow

### 1. Start Development Server
```bash
npm run start
```
This will start the Docusaurus development server at http://localhost:3000

### 2. Create New Module Structure
```bash
# Create module directories
mkdir -p docs/module-1 docs/module-2 docs/module-3 docs/module-4
mkdir -p docs/tutorials
```

### 3. Add Content Files
```bash
# Example: Create a new chapter in module 1
touch docs/module-1/introduction.md
```

### 4. Update Sidebar Configuration
Edit `sidebars.js` to reflect the new hierarchical structure:
```javascript
module.exports = {
  docs: [
    {
      type: 'category',
      label: 'Module 1',
      items: ['module-1/introduction', 'module-1/chapter-1'],
      collapsed: false,
    },
    // ... other modules
  ],
};
```

## Upgrade Process

### Phase 1: Foundation Update
1. Update Docusaurus dependencies:
   ```bash
   npm install @docusaurus/core@latest @docusaurus/preset-classic@latest
   ```

2. Verify build works:
   ```bash
   npm run build
   ```

### Phase 2: Content Migration
1. Create the new modular structure:
   ```bash
   # Create module directories
   mkdir -p docs/module-{1..4} docs/tutorials
   ```

2. Migrate content from existing files to new structure

3. Update `sidebars.js` to use hierarchical navigation

### Phase 3: Styling
1. Customize CSS in `src/css/custom.css`
2. Add new CSS files as needed (`components.css`, `design-system.css`)

## Key Commands

- `npm run start` - Start development server
- `npm run build` - Build static files for production
- `npm run serve` - Serve built files locally for testing
- `npm run deploy` - Deploy to GitHub Pages (if configured)

## Testing Checklist

- [ ] Development server starts without errors
- [ ] All internal links work correctly
- [ ] Navigation works as expected
- [ ] Responsive design functions on mobile
- [ ] Search functionality works
- [ ] Images and assets load correctly
- [ ] Custom CSS is applied properly

## Common Issues & Solutions

### Broken Links
- Check that all relative links are updated for new file locations
- Verify sidebar configuration matches file paths

### Build Errors
- Ensure all markdown files have proper frontmatter
- Check for syntax errors in configuration files

### Missing Assets
- Verify static assets are in the `/static` directory
- Update asset paths in content files if needed