import React, { useState } from 'react';

export default function ExpandableSection({title, children}) {
  const [expanded, setExpanded] = useState(false);

  return (
    <div className="expandable-section">
      <button 
        onClick={() => setExpanded(!expanded)}
        className="expandable-header"
        style={{
          width: '100%',
          padding: '10px',
          textAlign: 'left',
          background: '#f0f8ff',
          border: '1px solid #4CAF50',
          cursor: 'pointer',
          fontWeight: 'bold'
        }}
      >
        {expanded ? '▼ ' : '▶ '} {title}
      </button>
      {expanded && (
        <div 
          className="expandable-content"
          style={{
            padding: '15px',
            border: '1px solid #4CAF50',
            borderTop: 'none',
            background: '#f9f9f9'
          }}
        >
          {children}
        </div>
      )}
    </div>
  );
}