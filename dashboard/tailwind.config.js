/** @type {import('tailwindcss').Config} */

// ── AetherScan "obsidian" theme ────────────────────────────────────────────
// A deliberately near-monochrome, very dark palette. The bright slate/cyan
// utility classes scattered across the components are remapped here so the
// whole UI shifts to stylish black without touching every file:
//   • slate  → cool near-black → near-white neutral ramp (structure)
//   • cyan/blue/sky → one restrained cold "steel" accent (a whisper of life)
//   • green/emerald, amber/yellow/orange, red/rose → heavily MUTED semantics
//     (still readable for connected / warning / error, but desaturated)
//   • purple → neutral gray
const neutral = {
  50: '#f5f6f7', 100: '#e8e9ec', 200: '#c9cbd1', 300: '#9b9ea7',
  400: '#6a6d77', 500: '#474a52', 600: '#2d2f36', 700: '#1c1d22',
  800: '#131418', 900: '#0a0b0d', 950: '#050506',
}
const steel = {
  50: '#eef2f5', 100: '#dde4ea', 200: '#bcc8d2', 300: '#9aabb9',
  400: '#7c90a1', 500: '#617686', 600: '#4a5b69', 700: '#36434e',
  800: '#252e36', 900: '#171c21', 950: '#0c0f12',
}
const sage = {
  300: '#aebbae', 400: '#8a9b8a', 500: '#6c7d6c', 600: '#54634f',
}
const dust = {
  300: '#d4c8ad', 400: '#b6a47b', 500: '#8e7f5e', 600: '#6e6246',
}
const ember = {
  300: '#dba0a0', 400: '#c67070', 500: '#a84f4f', 600: '#893e3e',
}

module.exports = {
  content: ['./src/**/*.{js,ts,jsx,tsx,mdx}'],
  theme: {
    extend: {
      colors: {
        accent: { DEFAULT: steel[400], light: steel[300], dark: steel[600] },
        surface: { DEFAULT: neutral[800], light: neutral[700], dark: neutral[950] },
        // Remap the built-in families the components already use:
        slate: neutral,
        gray: neutral,
        zinc: neutral,
        neutral: neutral,
        cyan: steel,
        blue: steel,
        sky: steel,
        indigo: steel,
        purple: neutral,
        violet: neutral,
        green: sage,
        emerald: sage,
        teal: sage,
        amber: dust,
        yellow: dust,
        orange: dust,
        red: ember,
        rose: ember,
      },
      backdropBlur: { xs: '2px' },
      animation: {
        'pulse-slow': 'pulse 3s cubic-bezier(0.4, 0, 0.6, 1) infinite',
        'spin-slow': 'spin 3s linear infinite',
      },
    },
  },
  plugins: [],
}
