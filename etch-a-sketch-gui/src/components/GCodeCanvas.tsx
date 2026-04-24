import React, { useEffect, useRef, useState } from "react";
import { Box, Flex, Spinner, Text } from "@chakra-ui/react";

interface Move {
  draw: boolean;
  x: number;
  y: number;
}

function parseGcode(text: string): Move[] {
  const moves: Move[] = [];
  let curX = 0;
  let curY = 0;
  for (const raw of text.split("\n")) {
    const line = raw.trim().toUpperCase().split(";")[0];
    if (!line) continue;
    const isDraw = line.startsWith("G1") || line.startsWith("G01");
    const isTravel =
      line.startsWith("G0 ") ||
      line.startsWith("G00") ||
      line === "G0";
    if (!isDraw && !isTravel) continue;
    const xm = line.match(/X(-?\d+(?:\.\d+)?)/);
    const ym = line.match(/Y(-?\d+(?:\.\d+)?)/);
    if (xm) curX = parseFloat(xm[1]);
    if (ym) curY = parseFloat(ym[1]);
    moves.push({ draw: isDraw, x: curX, y: curY });
  }
  return moves;
}

function render(canvas: HTMLCanvasElement, moves: Move[]) {
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.offsetWidth || 560;
  const H = canvas.offsetHeight || 380;
  canvas.width = W * dpr;
  canvas.height = H * dpr;
  const ctx = canvas.getContext("2d")!;
  ctx.scale(dpr, dpr);

  // Background — dark with subtle radial glow for depth
  ctx.fillStyle = "#12121f";
  ctx.fillRect(0, 0, W, H);
  const grd = ctx.createRadialGradient(W / 2, H / 2, 0, W / 2, H / 2, Math.max(W, H) * 0.7);
  grd.addColorStop(0, "rgba(79,156,245,0.04)");
  grd.addColorStop(1, "rgba(0,0,0,0)");
  ctx.fillStyle = grd;
  ctx.fillRect(0, 0, W, H);

  if (moves.length === 0) {
    ctx.fillStyle = "#555";
    ctx.font = "14px system-ui";
    ctx.textAlign = "center";
    ctx.fillText("No moves found", W / 2, H / 2);
    return;
  }

  const xs = moves.map(m => m.x);
  const ys = moves.map(m => m.y);
  const minX = Math.min(...xs), maxX = Math.max(...xs);
  const minY = Math.min(...ys), maxY = Math.max(...ys);
  const rangeX = maxX - minX || 1;
  const rangeY = maxY - minY || 1;

  const pad = 32;
  const scale = Math.min((W - pad * 2) / rangeX, (H - pad * 2) / rangeY);
  const offX = (W - rangeX * scale) / 2;
  const offY = (H - rangeY * scale) / 2;

  const toX = (x: number) => offX + (x - minX) * scale;
  const toY = (y: number) => H - offY - (y - minY) * scale;

  ctx.lineCap = "round";
  ctx.lineJoin = "round";

  // Group consecutive moves of same type into path segments for perf
  type Seg = { draw: boolean; pts: [number, number][] };
  const segs: Seg[] = [];
  let cur: Seg | null = null;
  for (let i = 0; i < moves.length; i++) {
    const m = moves[i];
    if (!cur || cur.draw !== m.draw) {
      if (cur) segs.push(cur);
      cur = { draw: m.draw, pts: [] };
      if (i > 0) cur.pts.push([toX(moves[i - 1].x), toY(moves[i - 1].y)]);
    }
    cur.pts.push([toX(m.x), toY(m.y)]);
  }
  if (cur) segs.push(cur);

  // Travel moves: barely visible blue
  ctx.strokeStyle = "#4F9CF5";
  ctx.lineWidth = 0.5;
  ctx.globalAlpha = 0.06;
  for (const seg of segs) {
    if (seg.draw || seg.pts.length < 2) continue;
    ctx.beginPath();
    ctx.moveTo(seg.pts[0][0], seg.pts[0][1]);
    for (let i = 1; i < seg.pts.length; i++) ctx.lineTo(seg.pts[i][0], seg.pts[i][1]);
    ctx.stroke();
  }

  // Draw moves: amber glow
  ctx.globalAlpha = 1;
  ctx.lineWidth = 1.15;
  ctx.strokeStyle = "#FFC664";
  ctx.shadowColor = "rgba(255,198,100,0.35)";
  ctx.shadowBlur = 4;
  for (const seg of segs) {
    if (!seg.draw || seg.pts.length < 2) continue;
    ctx.beginPath();
    ctx.moveTo(seg.pts[0][0], seg.pts[0][1]);
    for (let i = 1; i < seg.pts.length; i++) ctx.lineTo(seg.pts[i][0], seg.pts[i][1]);
    ctx.stroke();
  }
  ctx.shadowBlur = 0;

  // Subtle corner brackets to frame the drawing
  const bx = offX - 10, by = offY - 10;
  const bw = rangeX * scale + 20, bh = rangeY * scale + 20;
  const bLen = 16;
  ctx.strokeStyle = "rgba(255,198,100,0.25)";
  ctx.lineWidth = 1.5;
  ctx.globalAlpha = 1;
  [[bx, by, 1, 1], [bx + bw, by, -1, 1], [bx, by + bh, 1, -1], [bx + bw, by + bh, -1, -1]].forEach(
    ([cx, cy, sx, sy]) => {
      ctx.beginPath();
      ctx.moveTo(cx as number, (cy as number) + (sy as number) * bLen);
      ctx.lineTo(cx as number, cy as number);
      ctx.lineTo((cx as number) + (sx as number) * bLen, cy as number);
      ctx.stroke();
    }
  );
}

interface Props {
  url: string;
}

const GCodeCanvas: React.FC<Props> = ({ url }) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const wrapperRef = useRef<HTMLDivElement>(null);
  const [state, setState] = useState<"loading" | "done" | "error">("loading");

  useEffect(() => {
    setState("loading");
    fetch(url)
      .then(r => {
        if (!r.ok) throw new Error("fetch failed");
        return r.text();
      })
      .then(text => {
        const moves = parseGcode(text);
        requestAnimationFrame(() => {
          if (canvasRef.current) {
            render(canvasRef.current, moves);
            setState("done");
          }
        });
      })
      .catch(() => setState("error"));
  }, [url]);

  return (
    <Box ref={wrapperRef} w="100%" h="100%" minH="340px" bg="#12121f" borderRadius="12px" overflow="hidden" position="relative">
      <canvas
        ref={canvasRef}
        style={{ width: "100%", height: "100%", display: "block" }}
      />
      {state === "loading" && (
        <Flex position="absolute" inset={0} align="center" justify="center" bg="#12121f">
          <Spinner color="#FFC664" size="lg" />
        </Flex>
      )}
      {state === "error" && (
        <Flex position="absolute" inset={0} align="center" justify="center" bg="#12121f">
          <Text color="#666" fontSize="13px">Failed to load GCode</Text>
        </Flex>
      )}
    </Box>
  );
};

export default GCodeCanvas;
