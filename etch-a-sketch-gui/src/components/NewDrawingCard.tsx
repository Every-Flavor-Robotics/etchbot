import React, { useState, useRef, useEffect, ChangeEvent } from "react";
import { Box, Flex, Text, Button, Select, NumberInput, NumberInputField, useToast } from "@chakra-ui/react";
import axios from "axios";
import { colors, card } from "../theme";

const API_URL = import.meta.env.VITE_API_URL || "http://localhost:5010";

interface Props { etchbotName: string; }

const NewDrawingCard: React.FC<Props> = ({ etchbotName }) => {
  const [pipeline, setPipeline] = useState(() => localStorage.getItem("pipeline") || "vtracer");
  const [framerate, setFramerate] = useState(() => parseInt(localStorage.getItem("framerate") || "8"));
  const [cameraState, setCameraState] = useState<"loading" | "ready" | "error">("loading");
  const [loading, setLoading] = useState(false);
  const videoRef = useRef<HTMLVideoElement>(null);
  const streamRef = useRef<MediaStream | null>(null);
  const fileInputRef = useRef<HTMLInputElement>(null);
  const toast = useToast();

  useEffect(() => {
    let active = true;
    navigator.mediaDevices?.getUserMedia({ video: true })
      .then(stream => {
        if (!active) { stream.getTracks().forEach(t => t.stop()); return; }
        streamRef.current = stream;
        if (videoRef.current) {
          videoRef.current.srcObject = stream;
          videoRef.current.play()
            .then(() => { if (active) setCameraState("ready"); })
            .catch(() => { if (active) setCameraState("error"); });
        }
      })
      .catch(() => { if (active) setCameraState("error"); });
    return () => {
      active = false;
      streamRef.current?.getTracks().forEach(t => t.stop());
    };
  }, []);

  useEffect(() => { localStorage.setItem("pipeline", pipeline); }, [pipeline]);
  useEffect(() => { localStorage.setItem("framerate", framerate.toString()); }, [framerate]);

  const upload = async (file: File) => {
    setLoading(true);
    const form = new FormData();
    form.append("file", file, file.name.replace(/ /g, "_"));
    form.append("pipeline", pipeline);
    form.append("framerate", framerate.toString());
    try {
      await axios.post(`${API_URL}/upload/${etchbotName}`, form, {
        headers: { "Content-Type": "multipart/form-data" },
      });
      toast({ title: "Upload successful!", status: "success", duration: 3000, isClosable: true });
    } catch {
      toast({ title: "Upload failed.", status: "error", duration: 3000, isClosable: true });
    } finally {
      setLoading(false);
    }
  };

  const handleCapture = () => {
    const video = videoRef.current;
    if (!video) return;
    const canvas = document.createElement("canvas");
    canvas.width = video.videoWidth || 640;
    canvas.height = video.videoHeight || 480;
    canvas.getContext("2d")?.drawImage(video, 0, 0);
    canvas.toBlob(blob => {
      if (!blob) return;
      upload(new File([blob], `capture_${Date.now()}.jpg`, { type: "image/jpeg" }));
    }, "image/jpeg", 0.92);
  };

  const handleFileChange = (e: ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (file) upload(file);
    e.target.value = "";
  };

  return (
    <Box
      bg={colors.blue}
      borderRadius={card.radius}
      p="16px"
      position="relative"
      overflow="hidden"
      display="flex"
      flexDirection="column"
    >
      {/* Decorative circles */}
      <Box position="absolute" top="-20px" right="-20px" w="80px" h="80px" bg="rgba(255,255,255,0.1)" borderRadius="50%" />
      <Box position="absolute" bottom="-30px" left="-10px" w="60px" h="60px" bg="rgba(255,255,255,0.08)" borderRadius="50%" />

      <Text color="rgba(255,255,255,0.85)" fontSize="10px" fontWeight="800" textTransform="uppercase" letterSpacing="1px" mb="12px">
        📷 New Drawing
      </Text>

      {/* Camera viewfinder */}
      <Box
        bg="rgba(0,0,0,0.25)"
        borderRadius="14px"
        flex="1"
        minH="260px"
        display="flex"
        alignItems="center"
        justifyContent="center"
        border="2px dashed rgba(255,255,255,0.4)"
        mb="12px"
        overflow="hidden"
        position="relative"
      >
        {/* Always render video so ref is available */}
        <video
          ref={videoRef}
          autoPlay
          muted
          playsInline
          style={{
            width: "100%",
            height: "100%",
            objectFit: "cover",
            borderRadius: "12px",
            display: cameraState === "ready" ? "block" : "none",
            transform: "scaleX(-1)",
          }}
        />
        {cameraState === "loading" && (
          <Text color="rgba(255,255,255,0.6)" fontSize="10px" fontWeight="600">Starting camera…</Text>
        )}
        {cameraState === "error" && (
          <Flex direction="column" align="center" gap="6px">
            <Text color="rgba(255,255,255,0.6)" fontSize="10px" fontWeight="600">Camera unavailable</Text>
            <Text color="rgba(255,255,255,0.4)" fontSize="9px">Use Upload below</Text>
          </Flex>
        )}
      </Box>

      {/* Capture / Upload buttons */}
      <Flex gap="8px" mb="10px">
        <Button
          flex={1}
          bg="white"
          color={colors.blue}
          fontSize="10px"
          fontWeight="800"
          borderRadius="12px"
          h="36px"
          isDisabled={cameraState !== "ready" || loading}
          onClick={handleCapture}
          _hover={{ bg: "#f0f6ff" }}
        >
          📷 Capture
        </Button>
        <Button
          flex={1}
          bg="rgba(255,255,255,0.2)"
          color="white"
          fontSize="10px"
          fontWeight="700"
          borderRadius="12px"
          h="36px"
          border="2px solid rgba(255,255,255,0.4)"
          isDisabled={loading}
          onClick={() => fileInputRef.current?.click()}
          _hover={{ bg: "rgba(255,255,255,0.3)" }}
        >
          ↑ Upload
        </Button>
        <input ref={fileInputRef} type="file" style={{ display: "none" }} onChange={handleFileChange} />
      </Flex>

      {/* Settings row */}
      <Flex gap="8px">
        <Box flex={1} bg="rgba(255,255,255,0.15)" borderRadius="10px" p="8px">
          <Text color="rgba(255,255,255,0.7)" fontSize="8px" fontWeight="700" textTransform="uppercase" letterSpacing="0.5px" mb="4px">Pipeline</Text>
          <Select
            value={pipeline}
            onChange={e => setPipeline(e.target.value)}
            size="xs"
            bg="transparent"
            border="none"
            color="white"
            fontWeight="700"
            fontSize="11px"
            p={0}
            _focus={{ boxShadow: "none" }}
          >
            <option value="vtracer" style={{ color: colors.ink }}>vtracer</option>
            <option value="potrace" style={{ color: colors.ink }}>potrace</option>
          </Select>
        </Box>
        <Box flex={1} bg="rgba(255,255,255,0.15)" borderRadius="10px" p="8px">
          <Text color="rgba(255,255,255,0.7)" fontSize="8px" fontWeight="700" textTransform="uppercase" letterSpacing="0.5px" mb="4px">Framerate</Text>
          <NumberInput value={framerate} min={1} max={60} size="xs" onChange={v => setFramerate(parseInt(v) || 8)}>
            <NumberInputField
              border="none"
              p={0}
              color="white"
              fontWeight="700"
              fontSize="11px"
              bg="transparent"
              _focus={{ boxShadow: "none" }}
            />
          </NumberInput>
        </Box>
      </Flex>
    </Box>
  );
};

export default NewDrawingCard;
