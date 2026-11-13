// Render an SVG to PDF (vector, fonts embedded) and PNG (2x) with Playwright.
// Usage: node render_svg.js figure.svg   -> figure.pdf, figure.png
const { chromium } = require('playwright');
const fs = require('fs');
(async () => {
  for (const svgPath of process.argv.slice(2)) {
    const svg = fs.readFileSync(svgPath, 'utf8');
    const W = +svg.match(/width="(\d+)"/)[1], H = +svg.match(/height="(\d+)"/)[1];
    const html = `<html><head><style>@page{size:${W}px ${H}px;margin:0}html,body{margin:0;padding:0}</style></head><body>${svg}</body></html>`;
    const browser = await chromium.launch();
    const page = await browser.newPage({ viewport: { width: W, height: H }, deviceScaleFactor: 2 });
    await page.setContent(html);
    await page.waitForTimeout(300);
    const stem = svgPath.replace(/\.svg$/, '');
    await page.screenshot({ path: stem + '.png', clip: { x: 0, y: 0, width: W, height: H } });
    await page.pdf({ path: stem + '.pdf', width: W + 'px', height: H + 'px', printBackground: true,
                     margin: { top: 0, bottom: 0, left: 0, right: 0 }, pageRanges: '1' });
    await browser.close();
    console.log('rendered', stem);
  }
})();
