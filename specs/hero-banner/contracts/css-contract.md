# CSS Contract: Hero Banner Component

## Component: HeroBanner

### Purpose
Defines the CSS interface and styling specifications for the responsive hero banner component with fluid scaling across 7 screen tiers.

### CSS Module Interface

```css
/* Main banner container */
.heroBanner {
  /* Required properties */
  position: relative;
  width: 100%;
  min-height: 50vh;
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 2rem;

  /* Background gradient that changes by screen size */
  background: linear-gradient(135deg, #0f0c29, #302b63, #24243e); /* XXL default */

  /* Animation with responsive duration */
  animation: fadeIn 1.5s ease-out;
}

/* Content wrapper */
.content {
  max-width: var(--hero-max-width, 1200px);
  text-align: center;
  z-index: 1;
}

/* Title styling with fluid typography */
.title {
  font-size: clamp(2rem, 4vw, 4rem);
  font-weight: 700;
  line-height: 1.2;
  margin: 0 0 1rem 0;
  color: var(--hero-title-color, white);
}

/* Subtitle styling with fluid typography */
.subtitle {
  font-size: clamp(1.2rem, 2.5vw, 2rem);
  font-weight: 500;
  line-height: 1.3;
  margin: 0 0 1.5rem 0;
  color: var(--hero-subtitle-color, rgba(255, 255, 255, 0.9));
}

/* Description styling with fluid typography */
.description {
  font-size: clamp(1rem, 2vw, 1.2rem);
  line-height: 1.5;
  margin: 0 0 2rem 0;
  color: var(--hero-description-color, rgba(255, 255, 255, 0.8));
}

/* Buttons container */
.buttons {
  display: flex;
  gap: 1rem;
  justify-content: center;
  flex-wrap: wrap;
}

/* Primary button styling */
.primaryButton {
  display: inline-block;
  padding: 0.75rem 1.5rem;
  min-height: 44px;
  min-width: 44px;
  font-size: 1rem;
  font-weight: 600;
  text-decoration: none;
  border-radius: 4px;
  background-color: var(--hero-primary-button-bg, #0072ce);
  color: var(--hero-primary-button-color, white);
  transition: all 0.3s ease;
}

/* Secondary button styling */
.secondaryButton {
  display: inline-block;
  padding: 0.75rem 1.5rem;
  min-height: 44px;
  min-width: 44px;
  font-size: 1rem;
  font-weight: 600;
  text-decoration: none;
  border-radius: 4px;
  background-color: transparent;
  color: var(--hero-secondary-button-color, white);
  border: 2px solid var(--hero-secondary-button-border, white);
  transition: all 0.3s ease;
}

/* Animation keyframes */
@keyframes fadeIn {
  from {
    opacity: 0;
    transform: translateY(20px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}
```

### Responsive Breakpoints

#### XXL Tier (1600px+)
- **Selector**: `.heroBanner` (base) with media query
- **Background**: Deep space gradient (`#0f0c29`, `#302b63`, `#24243e`)
- **Typography**: Largest clamp values
- **Animation**: Longest duration (2s)

#### XL Tier (1200px - 1599px)
- **Selector**: `@media (min-width: 1200px) and (max-width: 1599px)`
- **Background**: Ocean gradient (`#0f4c75`, `#3282b8`, `#4a90e2`)
- **Typography**: Large clamp values
- **Animation**: Long duration (1.8s)

#### LG Tier (992px - 1199px)
- **Selector**: `@media (min-width: 992px) and (max-width: 1199px)`
- **Background**: Royal gradient (`#536976`, `#292e49`, `#1a1a2e`)
- **Typography**: Medium-large clamp values
- **Animation**: Medium-long duration (1.5s)

#### MD Tier (768px - 991px)
- **Selector**: `@media (min-width: 768px) and (max-width: 991px)`
- **Background**: Deep sea gradient (`#0f3460`, `#1a1f3d`, `#2c3e50`)
- **Typography**: Medium clamp values
- **Animation**: Medium duration (1.3s)

#### SM Tier (576px - 767px)
- **Selector**: `@media (min-width: 576px) and (max-width: 767px)`
- **Background**: Forest gradient (`#2d5016`, `#42632d`, `#587a43`)
- **Typography**: Medium-small clamp values
- **Animation**: Medium-short duration (1.2s)

#### XS Tier (480px - 575px)
- **Selector**: `@media (min-width: 480px) and (max-width: 575px)`
- **Background**: Magenta gradient (`#8a2a5b`, `#c24077`, `#e85d88`)
- **Typography**: Small clamp values
- **Animation**: Short duration (1s)

#### XXS Tier (<479px)
- **Selector**: `@media (max-width: 479px)`
- **Background**: Crimson gradient (`#8b0000`, `#b22222`, `#dc143c`)
- **Typography**: Smallest clamp values
- **Animation**: Shortest duration (0.8s)

### Accessibility Features

#### Reduced Motion Support
- **Selector**: `@media (prefers-reduced-motion: reduce)`
- **Behavior**: Disable animations, reduce transitions
- **Required**: Must respect user's motion preferences

#### High Contrast Support
- **Selector**: `@media (prefers-contrast: high)`
- **Behavior**: Enhance color contrast, simplify gradients
- **Required**: Maintain readability in high contrast mode

#### Print Support
- **Selector**: `@media print`
- **Behavior**: Optimize for print output, remove animations
- **Required**: Ensure content remains readable when printed

### Validation Rules

#### CSS Properties
1. All typography must use `clamp()` for fluid scaling
2. Background gradients must change at each breakpoint
3. Animation durations must vary by screen size
4. Touch targets must be ≥44px (`min-height: 44px`, `min-width: 44px`)
5. Color contrast must meet WCAG AA standards

#### Performance Requirements
1. Animations must use `transform` and `opacity` to avoid layout thrashing
2. CSS selectors should be optimized to avoid performance bottlenecks
3. Media queries should be properly ordered (mobile-first approach recommended)

#### Accessibility Compliance
1. All interactive elements must be keyboard accessible
2. Focus indicators must be visible
3. Screen reader text must be available for non-visible content
4. Semantic HTML structure must be preserved