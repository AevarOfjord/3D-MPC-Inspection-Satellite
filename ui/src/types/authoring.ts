export interface ScanEndpointRef {
  scanId: string;
  endpoint: 'start' | 'end';
}

export type SelectedTransferEndpoint = ScanEndpointRef | null;
