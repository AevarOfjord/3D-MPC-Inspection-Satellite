export type SelectedProjectPlaneHandle = { scanId: string; handle: 'a' | 'b' } | null;
export type SelectedScanCenterHandle = { scanId: string } | null;
export type SelectedConnectorControl =
  | { connectorId: string; control: 'control1' | 'control2' }
  | null;
export type ConnectEndpoint = { scanId: string; endpoint: 'start' | 'end' } | null;
export type SelectedKeyLevelHandle =
  | {
      scanId: string;
      keyLevelId: string;
      handle: 'center' | 'rx_pos' | 'rx_neg' | 'ry_pos' | 'ry_neg';
    }
  | null;

export type CompileScanOptions = {
  silent?: boolean;
  signature?: string;
};
