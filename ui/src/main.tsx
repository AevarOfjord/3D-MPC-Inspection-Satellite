import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import './index.css'
import './styles/tokens.css'
import App from './App.tsx'
import { ErrorBoundary } from './components/ErrorBoundary.tsx'
import { FeedbackProvider } from './feedback/FeedbackProvider.tsx'

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <ErrorBoundary>
      <FeedbackProvider>
        <App />
      </FeedbackProvider>
    </ErrorBoundary>
  </StrictMode>,
)
