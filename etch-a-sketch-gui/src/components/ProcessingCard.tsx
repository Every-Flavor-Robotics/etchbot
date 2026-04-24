import React, { useState, useEffect } from "react";
import { Box, Flex, Text, Spinner } from "@chakra-ui/react";
import axios from "axios";
import { colors, card } from "../theme";

const API_URL = import.meta.env.VITE_API_URL || "http://localhost:5010";

interface StatusData { state: string; current_drawing: string | null; }

interface Props { etchbotName: string; }

const VTRACER_STEPS = ["Aspect Ratio", "Vectorize", "GCode", "Optimize"];

const ProcessingCard: React.FC<Props> = ({ etchbotName }) => {
  const [status, setStatus] = useState<StatusData | null>(null);

  useEffect(() => {
    const fetch = async () => {
      try {
        const res = await axios.get<StatusData>(`${API_URL}/etchbot/${etchbotName}/status`);
        setStatus(res.data);
      } catch { /* silent */ }
    };
    fetch();
    const id = setInterval(fetch, 1000);
    return () => clearInterval(id);
  }, [etchbotName]);

  const isDrawing = status?.state === "DRAWING";
  const steps = VTRACER_STEPS;

  return (
    <Box bg={colors.amber} borderRadius={card.radius} p="16px" position="relative" overflow="hidden">
      <Box position="absolute" top="-15px" right="-15px" w="70px" h="70px" bg="rgba(255,255,255,0.15)" borderRadius="50%" />

      <Flex align="center" justify="space-between" mb="12px">
        <Text color="rgba(0,0,0,0.65)" fontSize="10px" fontWeight="800" textTransform="uppercase" letterSpacing="1px">⚡ Processing</Text>
        {isDrawing && <Spinner size="sm" color="white" />}
      </Flex>

      {!isDrawing ? (
        <Box bg="rgba(255,255,255,0.4)" borderRadius="12px" p="12px">
          <Text color="rgba(0,0,0,0.5)" fontSize="10px" fontWeight="600" textAlign="center">Idle — no active job</Text>
        </Box>
      ) : (
        <Flex flexDirection="column" gap="7px">
          {steps.map((step, i) => (
            <Flex
              key={step}
              align="center"
              gap="8px"
              bg={i === 0 ? "rgba(255,255,255,0.9)" : "rgba(255,255,255,0.4)"}
              borderRadius="12px"
              p="9px 12px"
              opacity={i === 0 ? 1 : 0.6}
              boxShadow={i === 0 ? "0 2px 8px rgba(0,0,0,0.1)" : undefined}
            >
              <Box
                w="22px"
                h="22px"
                bg={i === 0 ? colors.blue : "rgba(0,0,0,0.1)"}
                borderRadius="50%"
                display="flex"
                alignItems="center"
                justifyContent="center"
                flexShrink={0}
              >
                {i === 0 ? <Spinner size="xs" color="white" /> : null}
              </Box>
              <Text color={colors.ink} fontSize="10px" fontWeight={i === 0 ? 700 : 600}>{step}</Text>
            </Flex>
          ))}
        </Flex>
      )}

      {status?.current_drawing && (
        <Text color="rgba(0,0,0,0.5)" fontSize="9px" fontWeight="600" mt="8px">
          {status.current_drawing}
        </Text>
      )}
    </Box>
  );
};

export default ProcessingCard;
