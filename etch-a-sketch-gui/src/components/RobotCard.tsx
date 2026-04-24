import React, { useState, useEffect } from "react";
import { Box, Flex, Text, Button, useToast } from "@chakra-ui/react";
import axios from "axios";
import { colors, card, stateColors } from "../theme";

const API_URL = import.meta.env.VITE_API_URL || "http://localhost:5010";

interface RobotStatusData {
  state: string;
  current_drawing: string | null;
  camera_connected: boolean;
  camera_recording: boolean;
  recording_mode: boolean;
  paused: boolean;
  cooldown_remaining: number;
}

interface Props { etchbotName: string; }

const RobotCard: React.FC<Props> = ({ etchbotName }) => {
  const [status, setStatus] = useState<RobotStatusData | null>(null);
  const toast = useToast();

  useEffect(() => {
    const fetch = async () => {
      try {
        const res = await axios.get<RobotStatusData>(`${API_URL}/etchbot/${etchbotName}/status`);
        setStatus(res.data);
      } catch { /* silent */ }
    };
    fetch();
    const id = setInterval(fetch, 1000);
    return () => clearInterval(id);
  }, [etchbotName]);

  const handlePauseResume = async () => {
    try {
      const endpoint = status?.paused ? "resume" : "pause";
      await axios.post(`${API_URL}/etchbot/${etchbotName}/${endpoint}`);
      toast({ title: status?.paused ? "Robot resumed." : "Robot paused.", status: "success", duration: 2000, isClosable: true });
    } catch {
      toast({ title: "Failed to pause/resume robot.", status: "error", duration: 2000, isClosable: true });
    }
  };

  const handleClearError = async () => {
    try {
      await axios.post(`${API_URL}/etchbot/${etchbotName}/clear_error`);
      toast({ title: "Error cleared.", status: "success", duration: 2000, isClosable: true });
    } catch {
      toast({ title: "Failed to clear error.", status: "error", duration: 2000, isClosable: true });
    }
  };

  return (
    <Box bg={colors.white} borderRadius={card.radius} p="14px" border={card.whiteBorder}>
      <Flex align="center" justify="space-between" mb="10px">
        <Text color={colors.ink} fontSize="10px" fontWeight="800" textTransform="uppercase" letterSpacing="1px">🤖 Robot</Text>
        {status && (
          <Box
            bg={`${stateColors[status.state] || "#9ca3af"}22`}
            border={`1.5px solid ${stateColors[status.state] || "#9ca3af"}`}
            borderRadius="20px"
            px="10px"
            py="3px"
            display="flex"
            alignItems="center"
            gap="5px"
          >
            <Box w="7px" h="7px" borderRadius="50%" bg={stateColors[status.state] || "#9ca3af"} />
            <Text fontSize="9px" fontWeight="700" color={stateColors[status.state] || "#9ca3af"}>{status.state}</Text>
          </Box>
        )}
      </Flex>

      {status && (
        <Box mb="10px">
          <Text fontSize="10px" color="#888" mb="2px">Drawing: <Text as="span" fontWeight="700" color={colors.ink}>{status.current_drawing || "—"}</Text></Text>
          <Text fontSize="10px" color="#888">Cooldown: <Text as="span" fontWeight="700" color={colors.ink}>{status.cooldown_remaining.toFixed(1)}s</Text></Text>
        </Box>
      )}

      <Flex gap="8px">
        <Button
          flex={1}
          bg={status?.paused ? "#22c55e" : colors.blue}
          color="white"
          fontSize="10px"
          fontWeight="700"
          borderRadius="12px"
          h="34px"
          onClick={handlePauseResume}
          _hover={{ opacity: 0.9 }}
        >
          {status?.paused ? "▶ Draw Next" : (status?.state === "DRAWING" ? "⏸ Pause" : "▶ Draw Next")}
        </Button>
        <Button
          flex={1}
          bg="#fff0ef"
          color={colors.coral}
          fontSize="10px"
          fontWeight="700"
          borderRadius="12px"
          h="34px"
          border={`2px solid ${colors.coral}`}
          isDisabled={status?.state !== "ERROR"}
          opacity={status?.state === "ERROR" ? 1 : 0.5}
          onClick={handleClearError}
          _hover={{ bg: "#ffe4e1" }}
        >
          Clear Error
        </Button>
      </Flex>
    </Box>
  );
};

export default RobotCard;
