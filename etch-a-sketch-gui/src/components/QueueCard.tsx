import React, { useState, useEffect } from "react";
import { Box, Flex, Text, IconButton } from "@chakra-ui/react";
import { FaImages, FaTrash } from "react-icons/fa";
import axios from "axios";
import { colors, card } from "../theme";
import StepViewerModal from "./StepViewerModal";

const API_URL = import.meta.env.VITE_API_URL || "http://localhost:5010";

interface QueueItem { name: string; length: number; ready: boolean; }
interface QueueStatusResponse { queue: QueueItem[]; }
interface Props { etchbotName: string; }

function nameToStem(name: string): string {
  return name.replace(/\.[^.]+$/, "");
}

const QueueCard: React.FC<Props> = ({ etchbotName }) => {
  const [queue, setQueue] = useState<QueueItem[]>([]);
  const [viewerOpen, setViewerOpen] = useState(false);
  const [viewerStem, setViewerStem] = useState("");

  const fetchQueue = async () => {
    try {
      const res = await axios.get<QueueStatusResponse>(`${API_URL}/etchbot/queue`, { params: { name: etchbotName } });
      setQueue(res.data.queue);
    } catch { /* silent */ }
  };

  useEffect(() => {
    fetchQueue();
    const id = setInterval(fetchQueue, 5000);
    return () => clearInterval(id);
  }, [etchbotName]);

  const handleDelete = async (index: number) => {
    try {
      await axios.delete(`${API_URL}/etchbot/${etchbotName}/queue/${index}`);
      fetchQueue();
    } catch { /* silent */ }
  };

  return (
    <>
      <Box bg={colors.coral} borderRadius={card.radius} p="14px" position="relative" overflow="hidden">
        <Box position="absolute" bottom="-20px" right="-20px" w="70px" h="70px" bg="rgba(255,255,255,0.1)" borderRadius="50%" />

        <Text color="rgba(255,255,255,0.85)" fontSize="10px" fontWeight="800" textTransform="uppercase" letterSpacing="1px" mb="10px">
          🎨 Queue
        </Text>

        {queue.length === 0 ? (
          <Box bg="rgba(255,255,255,0.2)" borderRadius="12px" p="10px">
            <Text color="rgba(255,255,255,0.7)" fontSize="10px" fontWeight="600" textAlign="center">Queue empty</Text>
          </Box>
        ) : (
          <Flex flexDirection="column" gap="6px">
            {queue.map((item, index) => (
              <Flex
                key={item.name}
                bg={item.ready ? "rgba(255,255,255,0.2)" : "rgba(255,255,255,0.15)"}
                borderRadius="12px"
                p="8px 12px"
                justify="space-between"
                align="center"
              >
                <Text color="white" fontSize="10px" fontWeight="700" flex={1} mr="8px" noOfLines={1}>{item.name}</Text>
                <Flex gap="5px" align="center" flexShrink={0}>
                  <Box bg="rgba(255,255,255,0.3)" borderRadius="20px" px="8px" py="2px">
                    <Text color="white" fontSize="8px" fontWeight="700">
                      {item.ready ? "Ready" : "Processing"}
                    </Text>
                  </Box>
                  {item.ready && (
                    <IconButton
                      aria-label="View steps"
                      icon={<FaImages />}
                      size="xs"
                      bg="rgba(255,255,255,0.9)"
                      color={colors.coral}
                      borderRadius="8px"
                      _hover={{ bg: "white" }}
                      onClick={() => { setViewerStem(nameToStem(item.name)); setViewerOpen(true); }}
                    />
                  )}
                  <IconButton
                    aria-label="Delete"
                    icon={<FaTrash />}
                    size="xs"
                    bg="rgba(0,0,0,0.2)"
                    color="rgba(255,255,255,0.8)"
                    borderRadius="8px"
                    _hover={{ bg: "rgba(0,0,0,0.4)", color: "white" }}
                    onClick={() => handleDelete(index)}
                  />
                </Flex>
              </Flex>
            ))}
          </Flex>
        )}
      </Box>

      <StepViewerModal
        isOpen={viewerOpen}
        onClose={() => setViewerOpen(false)}
        etchbotName={etchbotName}
        drawingStem={viewerStem}
      />
    </>
  );
};

export default QueueCard;
