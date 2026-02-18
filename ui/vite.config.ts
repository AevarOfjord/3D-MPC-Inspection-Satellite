import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  build: {
    // 3D vendor chunks are intentionally heavy in desktop-local mode.
    // Raise warning threshold after manual chunking so CI noise is reduced.
    chunkSizeWarningLimit: 800,
    rollupOptions: {
      output: {
        manualChunks(id) {
          if (id.includes('node_modules/react') || id.includes('node_modules/react-dom')) {
            return 'react'
          }
          if (id.includes('node_modules/three/examples') || id.includes('node_modules/three-stdlib')) {
            return 'three-extras'
          }
          if (id.includes('node_modules/@react-three/')) {
            return 'react-three'
          }
          if (id.includes('node_modules/three/src/renderers')) {
            return 'three-renderers'
          }
          if (id.includes('node_modules/three/src/math') || id.includes('node_modules/three/src/core')) {
            return 'three-foundation'
          }
          if (
            id.includes('node_modules/three/src/geometries') ||
            id.includes('node_modules/three/src/materials') ||
            id.includes('node_modules/three/src/objects') ||
            id.includes('node_modules/three/src/textures') ||
            id.includes('node_modules/three/src/lights') ||
            id.includes('node_modules/three/src/scenes')
          ) {
            return 'three-scene'
          }
          if (id.includes('node_modules/three/')) {
            return 'three-misc'
          }
          if (id.includes('node_modules/@dnd-kit/')) {
            return 'dnd'
          }
          if (id.includes('node_modules/lucide-react')) {
            return 'icons'
          }
          return undefined
        },
      },
    },
  },
  server: {
    proxy: {
      '/api': {
        target: 'http://localhost:8000',
        changeOrigin: true,
      },
      '/ws': {
        target: 'ws://localhost:8000',
        ws: true,
      },
    },
  },
})
