import React from 'react';

// Root wrapper component that adds the chat widget to all pages
// The ChatWidget will handle its own client-side rendering logic
export default function Root({ children }) {
  return (
    <>
      {children}
      <div id="chat-widget-root">
        <ChatWidgetLoader />
      </div>
    </>
  );
}

// Separate component to handle the dynamic loading of ChatWidget
function ChatWidgetLoader() {
  const [ChatWidgetComponent, setChatWidgetComponent] = React.useState(null);

  React.useEffect(() => {
    // Only import on client side
    if (typeof window !== 'undefined') {
      import('@site/src/components/ChatWidget/ChatWidget')
        .then(module => {
          setChatWidgetComponent(() => module.default);
        })
        .catch(error => {
          console.warn('ChatWidget failed to load:', error);
        });
    }
  }, []);

  if (ChatWidgetComponent) {
    return <ChatWidgetComponent />;
  }

  // Return null if not loaded (won't show anything until component loads)
  return null;
}