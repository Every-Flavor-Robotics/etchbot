import React, { useState, useEffect } from "react";
import { Box, Flex, Text, IconButton } from "@chakra-ui/react";
import { FaDownload, FaImages } from "react-icons/fa";
import axios from "axios";
import { colors, card } from "../theme";
import StepViewerModal from "./StepViewerModal";

const API_URL = import.meta.env.VITE_API_URL || "http://localhost:5010";

interface CompletedItem { name: string; length: number; ready: boolean; }
interface CompletedStatusResponse { completed: CompletedItem[]; }
interface Props { etchbotName: string; }

function nameToStem(name: string): string {
  return name.replace(/\.[^.]+$/, "");
}

const CompletedCard: React.FC<Props> = ({ etchbotName }) => {
  const [completed, setCompleted] = useState<CompletedItem[]>([]);
  const [viewerOpen, setViewerOpen] = useState(false);
  const [viewerStem, setViewerStem] = useState("");

  useEffect(() => {
    const fetch = async () => {
      try {
        const res = await axios.get<CompletedStatusResponse>(`${API_URL}/etchbot/completed`, { params: { name: etchbotName } });
        setCompleted(Object.values(res.data.completed));
      } catch { /* silent */ }
    };
    fetch();
    const id = setInterval(fetch, 5000);
    return () => clearInterval(id);
  }, [etchbotName]);

  const handleDownload = async (name: string, index: number) => {
    try {
      const res = await axios.post<Blob>(
        `${API_URL}/etchbot/${etchbotName}/download_zip`,
        { drawing_index: index },
        { responseType: "blob" }
      );
      const url = window.URL.createObjectURL(res.data);
      const a = document.createElement("a");
      a.href = url;
      a.setAttribute("download", `${name}.zip`);
      document.body.appendChild(a);
      a.click();
      a.remove();
    } catch { /* silent */ }
  };

  return (
    <>
      <Box bg={colors.white} borderRadius={card.radius} p="14px" border={card.whiteBorder}>
        <Text color={colors.ink} fontSize="10px" fontWeight="800" textTransform="uppercase" letterSpacing="1px" mb="10px">
          ✅ Completed
        </Text>

        {completed.length === 0 ? (
          <Box bg="#f7f8fc" borderRadius="12px" p="10px">
            <Text color="#aaa" fontSize="10px" fontWeight="600" textAlign="center">No completed drawings</Text>
          </Box>
        ) : (
          <Flex flexDirection="column" gap="6px">
            {completed.map((item, index) => (
              <Flex
                key={item.name}
                bg="#f7f8fc"
                borderRadius="12px"
                p="8px 12px"
                justify="space-between"
                align="center"
              >
                <Text color={colors.ink} fontSize="10px" fontWeight="600">{item.name}</Text>
                <Flex gap="4px">
                  <IconButton
                    aria-label="View steps"
                    icon={<FaImages />}
                    size="xs"
                    bg={colors.amber}
                    color="white"
                    borderRadius="8px"
                    _hover={{ opacity: 0.85 }}
                    onClick={() => { setViewerStem(nameToStem(item.name)); setViewerOpen(true); }}
                  />
                  <IconButton
                    aria-label="Download"
                    icon={<FaDownload />}
                    size="xs"
                    bg={colors.blue}
                    color="white"
                    borderRadius="8px"
                    _hover={{ opacity: 0.85 }}
                    onClick={() => handleDownload(item.name, index)}
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

export default CompletedCard;
