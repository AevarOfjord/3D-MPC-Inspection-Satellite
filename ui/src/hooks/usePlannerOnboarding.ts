import { useEffect, useMemo, useState } from 'react';

import {
  DEFAULT_COACHMARK_STATE,
  PLANNER_COACHMARKS_STORAGE_KEY,
  type CoachmarkId,
  type PlannerCoachmarkState,
} from '../types/plannerUx';

function parseCoachmarkState(raw: string | null): PlannerCoachmarkState {
  if (!raw) return DEFAULT_COACHMARK_STATE;
  try {
    const parsed = JSON.parse(raw) as Partial<PlannerCoachmarkState>;
    return {
      introSeen: Boolean(parsed.introSeen),
      neverShowAgain: Boolean(parsed.neverShowAgain),
      dismissedIds: Array.isArray(parsed.dismissedIds)
        ? parsed.dismissedIds.filter((id): id is CoachmarkId => typeof id === 'string')
        : [],
    };
  } catch {
    return DEFAULT_COACHMARK_STATE;
  }
}

export function usePlannerOnboarding() {
  const [state, setState] = useState<PlannerCoachmarkState>(() => {
    try {
      return parseCoachmarkState(window.localStorage.getItem(PLANNER_COACHMARKS_STORAGE_KEY));
    } catch {
      return DEFAULT_COACHMARK_STATE;
    }
  });
  const [tourOpen, setTourOpen] = useState(false);

  useEffect(() => {
    try {
      window.localStorage.setItem(PLANNER_COACHMARKS_STORAGE_KEY, JSON.stringify(state));
    } catch {
      // ignore storage write errors
    }
  }, [state]);

  const showIntroBanner = !state.introSeen && !state.neverShowAgain;

  const visibleCoachmarks = useMemo(
    () =>
      (['step_rail', 'templates', 'context_panel', 'validation', 'save_launch'] as CoachmarkId[])
        .filter((id) => !state.dismissedIds.includes(id)),
    [state.dismissedIds]
  );

  const startTour = () => {
    setState((prev) => ({ ...prev, introSeen: true }));
    setTourOpen(true);
  };

  const dismissIntro = () => {
    setState((prev) => ({ ...prev, introSeen: true }));
  };

  const setNeverShowAgain = () => {
    setState((prev) => ({ ...prev, introSeen: true, neverShowAgain: true }));
    setTourOpen(false);
  };

  const dismissCoachmark = (id: CoachmarkId) => {
    setState((prev) => {
      if (prev.dismissedIds.includes(id)) return prev;
      return { ...prev, dismissedIds: [...prev.dismissedIds, id] };
    });
  };

  return {
    state: {
      showIntroBanner,
      tourOpen,
      visibleCoachmarks,
      neverShowAgain: state.neverShowAgain,
    },
    actions: {
      startTour,
      dismissIntro,
      setNeverShowAgain,
      closeTour: () => setTourOpen(false),
      dismissCoachmark,
    },
  };
}
