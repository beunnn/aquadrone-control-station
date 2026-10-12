// Formats the handwritten firmware sources: clang-format (pinned via uv) for layout, then a small
// postprocessor for the two rules clang-format cannot express: blank lines around control blocks and
// bounded alignment of "=" in consecutive assignments.
//   node tools/format-code.mjs            format in place
//   node tools/format-code.mjs --check    report differences, exit 1 if any
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { execFileSync } from 'node:child_process';

const repo = fileURLToPath(new URL('../', import.meta.url));
const sourceDirs = ['firmware/control_box/main', 'firmware/tab5/main', 'firmware/components/common_types/include'];
const check = process.argv.includes('--check');
const maskedTokens = /\/\*[\s\S]*?\*\/|\/\/[^\n]*|"(?:\\[\s\S]|[^"\\])*"|'(?:\\[\s\S]|[^'\\])*'/g;

function maskLiterals(text) {
  return text.replace(maskedTokens, token => token.replace(/[^\n]/g, ' '));
}

function finishSpacing(text) {
  const lines = text.replace(/\r\n/g, '\n').split('\n');
  const masked = maskLiterals(text.replace(/\r\n/g, '\n')).split('\n');
  const separated = [];
  for (let index = 0; index < lines.length; index++) {
    const current = masked[index].trim();
    const previous = index > 0 ? masked[index - 1].trim() : '';
    const previousIsComment = index > 0 && lines[index - 1].trim().startsWith('//');
    const controlStart = /^(?:if|for|while|switch|do)\b/.test(current);
    const compactControl = /^(?:if|else|for|while|switch)\b.*\{.*\}$/.test(previous);
    const followsBlock = (previous === '}' || compactControl) && current !== '' &&
      !/^(?:else\b|while\b.*;$|[};,)#])/.test(current);
    if ((controlStart && previous !== '' && !previousIsComment && !/^(?:\{|else\b|#)/.test(previous)) || followsBlock) {
      if (separated.at(-1)?.trim()) separated.push('');
    }
    separated.push(lines[index]);
  }

  const maskedSeparated = maskLiterals(separated.join('\n')).split('\n');
  const output = [];
  let group = [];
  function flush() {
    const width = Math.max(0, ...group.map(item => item.left.length));
    output.push(...group.map(item => item.left.padEnd(width) + ' = ' + item.right));
    group = [];
  }
  for (let index = 0; index < separated.length; index++) {
    const line = separated[index];
    const code = maskedSeparated[index];
    const match = /(?<![<>=!+\-*/%&|^])=(?!=)/.exec(code);
    const assignment = match && /;\s*$/.test(code) &&
      !/^\s*(?:#|for\b|if\b|else\b|while\b|return\b|case\b|_Static_assert\b)/.test(code) && !/\{.*\}\s*;?\s*$/.test(code.slice(0, match.index));
    if (!assignment) {
      flush();
      output.push(line);
      continue;
    }
    const left = line.slice(0, match.index).trimEnd();
    const right = line.slice(match.index + 1).trimStart();
    const indent = /^\s*/.exec(left)[0].length;
    const widths = [...group.map(item => item.left.length), left.length];
    if (group.length && (group[0].indent !== indent || Math.max(...widths) - Math.min(...widths) > 6)) {
      flush();
      output.push('');
    }
    group.push({ left, right, indent });
  }
  flush();
  return output.join('\n').replace(/\n{3,}/g, '\n\n').replace(/\n*$/, '\n');
}

function verifySpacing() {
  const fixture = 'void example( void )\n{\n    int32_t value = 0;\n    if ( value == 0 )\n    {\n        value = 1;\n    }\n    use( value );\n    int32_t unusuallyLongName = 2;\n    int32_t shortName = 3;\n}\n';
  const result = finishSpacing(fixture);
  assert.match(result, /value = 0;\n\n    if/);
  assert.match(result, /}\n\n    use/);
  assert.match(result, /unusuallyLongName = 2;\n\n    int32_t shortName/);
  assert.equal(finishSpacing(result), result);
  assert.equal(finishSpacing('    text = "if ( value ) = 0x000000";\n'), '    text = "if ( value ) = 0x000000";\n');
  assert.equal(finishSpacing('    // why\n    if ( ready ) { go(); }\n'), '    // why\n    if ( ready ) { go(); }\n');
}

function formatSource(source, filename) {
  const formatted = execFileSync(process.platform === 'win32' ? 'uv.exe' : 'uv',
    ['tool', 'run', '--from', 'clang-format==19.1.7', 'clang-format', '--style=file', `--assume-filename=${filename}`],
    { input: source, encoding: 'utf8', maxBuffer: 4 * 1024 * 1024 });
  return finishSpacing(formatted);
}

verifySpacing();
const files = sourceDirs.flatMap(dir => fs.readdirSync(path.join(repo, dir)).filter(file => /\.(c|h)$/.test(file)).map(file => path.join(repo, dir, file)));
let differences = 0;
for (const filename of files) {
  const original = fs.readFileSync(filename, 'utf8');
  const result = formatSource(original, filename);
  if (result !== original) {
    differences++;
    if (check) console.error(`Formatting differs: ${path.relative(repo, filename)}`);
    else fs.writeFileSync(filename, result);
  }
}
console.log(`${files.length} handwritten files checked; ${differences} ${check ? 'differences' : 'formatted'}.`);
if (check && differences) process.exitCode = 1;
