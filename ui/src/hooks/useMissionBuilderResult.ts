export function buildMissionBuilderResult<
  TBaseState extends object,
  TScanState extends object,
  TMissionState extends object,
  TValidationState extends object,
  TDraftState extends object,
  THistoryState extends object,
  TBaseSetters extends object,
  TScanSetters extends object,
  TMissionSetters extends object,
  TValidationSetters extends object,
  TIoActions extends object,
  TScanProjectActions extends object,
  TInteractionActions extends object,
  TMissionActions extends object,
  TValidationActions extends object,
  TPersistenceActions extends object,
  TExecutionActions extends object,
  TScanPlaneActions extends object,
  TMiscActions extends object,
>(args: {
  baseState: TBaseState;
  scanState: TScanState;
  missionState: TMissionState;
  validationState: TValidationState;
  draftState: TDraftState;
  historyState: THistoryState;
  baseSetters: TBaseSetters;
  scanSetters: TScanSetters;
  missionSetters: TMissionSetters;
  validationSetters: TValidationSetters;
  ioActions: TIoActions;
  scanProjectActions: TScanProjectActions;
  interactionActions: TInteractionActions;
  missionActions: TMissionActions;
  validationActions: TValidationActions;
  persistenceActions: TPersistenceActions;
  executionActions: TExecutionActions;
  scanPlaneActions: TScanPlaneActions;
  miscActions: TMiscActions;
}) {
  return {
    state: {
      ...args.baseState,
      ...args.scanState,
      ...args.missionState,
      ...args.validationState,
      ...args.draftState,
      ...args.historyState,
    },
    setters: {
      ...args.baseSetters,
      ...args.scanSetters,
      ...args.missionSetters,
      ...args.validationSetters,
    },
    actions: {
      ...args.ioActions,
      ...args.scanProjectActions,
      ...args.interactionActions,
      ...args.missionActions,
      ...args.validationActions,
      ...args.persistenceActions,
      ...args.executionActions,
      ...args.scanPlaneActions,
      ...args.miscActions,
    },
  };
}
