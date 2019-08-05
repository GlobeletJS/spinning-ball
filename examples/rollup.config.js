import resolve from 'rollup-plugin-node-resolve';
import commonjs from 'rollup-plugin-commonjs'; // Yuck... avoid if possible
import json from 'rollup-plugin-json';

export default [
  {
    input: 'd3-world-atlas/main.js',
    plugins: [
      resolve(),
      commonjs(),
      json(),
    ],
    output: {
      file: 'd3-world-atlas/main.min.js',
      format: 'iife',
      name: 'example',
    },
  }
];
