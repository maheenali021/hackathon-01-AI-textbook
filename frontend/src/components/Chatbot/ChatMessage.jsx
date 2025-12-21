import React from 'react';

export const ChatMessage = ({ message }) => {
  const isUser = message.role === 'user';

  return (
    <div className={`message ${isUser ? 'user-message' : 'assistant-message'}`}>
      <div className="message-content">
        <div className="message-text">
          {message.content}
        </div>

        {message.sources && message.sources.length > 0 && (
          <div className="message-sources">
            <details>
              <summary>Sources ({message.sources.length})</summary>
              <ul>
                {message.sources.map((source, index) => (
                  <li key={index} className="source-item">
                    <strong>Source {index + 1}:</strong> {source.metadata?.title || source.metadata?.source || 'N/A'}
                    {source.metadata?.page && <span>, Page: {source.metadata.page}</span>}
                    {source.metadata?.section && <span>, Section: {source.metadata.section}</span>}
                  </li>
                ))}
              </ul>
            </details>
          </div>
        )}

        {message.isError && (
          <div className="error-message">
            <small>Please try rephrasing your question or try again later.</small>
          </div>
        )}
      </div>
    </div>
  );
};