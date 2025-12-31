# Quickstart: Hero Banner Implementation

## Prerequisites

- Node.js (v18 or higher)
- npm or yarn package manager
- Docusaurus project setup
- Basic knowledge of React and CSS

## Setup Environment

1. **Navigate to the frontend directory**:
   ```bash
   cd frontend-ai-book
   ```

2. **Install dependencies** (if needed):
   ```bash
   npm install
   ```

## Component Development Workflow

### 1. Create Component Directory
```bash
mkdir -p src/components/HeroBanner
```

### 2. Create Component Files
```bash
# Create the main component file
touch src/components/HeroBanner/HeroBanner.jsx

# Create the CSS module file
touch src/components/HeroBanner/HeroBanner.module.css

# Create the index file
touch src/components/HeroBanner/index.js
```

### 3. Basic Component Structure
```javascript
// src/components/HeroBanner/HeroBanner.jsx
import React from 'react';
import styles from './HeroBanner.module.css';

const HeroBanner = ({ title, subtitle, description, primaryButton, secondaryButton }) => {
  return (
    <header className={styles.heroBanner}>
      <div className={styles.content}>
        <h1 className={styles.title}>{title}</h1>
        {subtitle && <h2 className={styles.subtitle}>{subtitle}</h2>}
        {description && <p className={styles.description}>{description}</p>}
        <div className={styles.buttons}>
          {primaryButton && (
            <a href={primaryButton.url} className={styles.primaryButton}>
              {primaryButton.text}
            </a>
          )}
          {secondaryButton && (
            <a href={secondaryButton.url} className={styles.secondaryButton}>
              {secondaryButton.text}
            </a>
          )}
        </div>
      </div>
    </header>
  );
};

export default HeroBanner;
```

## Responsive Design Implementation

### 1. CSS Clamp() for Fluid Typography
```css
/* In HeroBanner.module.css */
.title {
  font-size: clamp(2rem, 4vw, 4rem);
  line-height: 1.2;
}

.subtitle {
  font-size: clamp(1.2rem, 2.5vw, 2rem);
  line-height: 1.3;
}

.description {
  font-size: clamp(1rem, 2vw, 1.2rem);
  line-height: 1.5;
}
```

### 2. Seven-Tier Breakpoint System
```css
/* XXL: 1600px+ */
@media (min-width: 1600px) {
  .heroBanner {
    background: linear-gradient(135deg, #0f0c29, #302b63, #24243e);
  }
}

/* XL: 1200px to 1599px */
@media (min-width: 1200px) and (max-width: 1599px) {
  .heroBanner {
    background: linear-gradient(135deg, #0f4c75, #3282b8, #4a90e2);
  }
}

/* LG: 992px to 1199px */
@media (min-width: 992px) and (max-width: 1199px) {
  .heroBanner {
    background: linear-gradient(135deg, #536976, #292e49, #1a1a2e);
  }
}

/* MD: 768px to 991px */
@media (min-width: 768px) and (max-width: 991px) {
  .heroBanner {
    background: linear-gradient(135deg, #0f3460, #1a1f3d, #2c3e50);
  }
}

/* SM: 576px to 767px */
@media (min-width: 576px) and (max-width: 767px) {
  .heroBanner {
    background: linear-gradient(135deg, #2d5016, #42632d, #587a43);
  }
}

/* XS: 480px to 575px */
@media (min-width: 480px) and (max-width: 575px) {
  .heroBanner {
    background: linear-gradient(135deg, #8a2a5b, #c24077, #e85d88);
  }
}

/* XXS: <479px */
@media (max-width: 479px) {
  .heroBanner {
    background: linear-gradient(135deg, #8b0000, #b22222, #dc143c);
  }
}
```

### 3. Animation with Responsive Duration
```css
.heroBanner {
  animation: fadeIn 1.5s ease-out;
}

/* Animation duration adjusts by screen size */
@media (min-width: 1200px) {
  .heroBanner {
    animation-duration: 2s;
  }
}

@media (max-width: 767px) {
  .heroBanner {
    animation-duration: 1s;
  }
}

/* Respect user's motion preferences */
@media (prefers-reduced-motion: reduce) {
  .heroBanner {
    animation: none;
  }
}
```

## Accessibility Implementation

### 1. Semantic HTML Structure
- Use `<header>` for the banner container
- Use `<h1>` for the main title
- Include proper heading hierarchy
- Add ARIA attributes where needed

### 2. Touch Target Requirements
```css
.primaryButton,
.secondaryButton {
  min-height: 44px;
  min-width: 44px;
  padding: 12px 24px;
}
```

### 3. WCAG AA Color Contrast
- Ensure minimum 4.5:1 contrast ratio for normal text
- Use 3:1 ratio for large text
- Test with accessibility tools

## Integration with Docusaurus

### 1. Update Homepage
```javascript
// In src/pages/index.js
import HeroBanner from '../components/HeroBanner';

const Home = () => {
  const heroProps = {
    title: "Physical AI Robotic Book",
    subtitle: "Advanced robotics and AI integration",
    description: "Learn about cutting-edge robotics, AI, and automation technologies.",
    primaryButton: {
      text: "Get Started",
      url: "/docs/intro"
    },
    secondaryButton: {
      text: "View Tutorials",
      url: "/docs/tutorials/setup-guide"
    }
  };

  return (
    <Layout title="Home" description="Physical AI Robotic Book">
      <HeroBanner {...heroProps} />
      {/* Rest of the page content */}
    </Layout>
  );
};
```

## Testing Checklist

- [ ] Component renders correctly at all 7 breakpoints
- [ ] Typography scales fluidly with viewport size
- [ ] Gradients change appropriately at each breakpoint
- [ ] Animations respect user's motion preferences
- [ ] Touch targets are ≥44px
- [ ] Color contrast meets WCAG AA standards
- [ ] Component is accessible to screen readers
- [ ] Print styles work correctly
- [ ] Performance is smooth (60fps animations)

## Common Issues & Solutions

### Performance Issues
- Use `transform` and `opacity` for animations to avoid layout thrashing
- Consider using `will-change` for elements that will be animated
- Optimize CSS selectors to avoid performance bottlenecks

### Responsive Design Issues
- Test on actual devices when possible
- Use browser dev tools to simulate different screen sizes
- Check that text remains readable at all sizes