import { useEffect } from 'react';
import { useThree } from '@react-three/fiber';
import { useViewportStore } from '../store/viewportStore';

export function CanvasRegistrar() {
  const { gl } = useThree();
  const setCanvas = useViewportStore(s => s.setCanvas);

  useEffect(() => {
    setCanvas(gl.domElement);
    return () => {
      setCanvas(null);
    };
  }, [gl, setCanvas]);

  return null;
}
