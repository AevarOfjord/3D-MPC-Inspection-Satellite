import { Component, type ErrorInfo, type ReactNode } from 'react';

interface Props {
  children: ReactNode;
  fallback?: ReactNode;
}

interface State {
  hasError: boolean;
  error: Error | null;
}

/**
 * Catches rendering errors in child components and displays
 * a fallback UI instead of crashing the entire application.
 */
export class ErrorBoundary extends Component<Props, State> {
  constructor(props: Props) {
    super(props);
    this.state = { hasError: false, error: null };
  }

  static getDerivedStateFromError(error: Error): State {
    return { hasError: true, error };
  }

  componentDidCatch(error: Error, info: ErrorInfo) {
    console.error('[ErrorBoundary]', error, info.componentStack);
  }

  render() {
    if (this.state.hasError) {
      return (
        this.props.fallback ?? (
          <div className="flex items-center justify-center h-full p-8">
            <div className="bg-red-900/20 border border-red-500/30 rounded-lg p-6 max-w-md">
              <h2 className="text-red-400 font-semibold mb-2">Something went wrong</h2>
              <p className="text-gray-300 text-sm font-mono">
                {this.state.error?.message ?? 'Unknown error'}
              </p>
              <button
                className="mt-4 px-4 py-2 bg-red-600 hover:bg-red-700 text-white rounded text-sm"
                onClick={() => this.setState({ hasError: false, error: null })}
              >
                Try Again
              </button>
            </div>
          </div>
        )
      );
    }

    return this.props.children;
  }
}
