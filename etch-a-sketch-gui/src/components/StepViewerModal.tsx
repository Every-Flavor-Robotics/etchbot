import React, { useState, useEffect } from "react";
import {
  Modal, ModalOverlay, ModalContent, ModalHeader, ModalBody, ModalCloseButton,
  Box, Flex, Text, Button, Spinner
} from "@chakra-ui/react";
import axios from "axios";
import { colors } from "../theme";
import GCodeCanvas from "./GCodeCanvas";

const API_URL = import.meta.env.VITE_API_URL || "http://localhost:5010";

interface StepFile { filename: string; ext: string; }
interface StepsResponse { steps: StepFile[]; }

interface Props {
  isOpen: boolean;
  onClose: () => void;
  etchbotName: string;
  drawingStem: string;
}

function formatStepName(filename: string): string {
  const match = filename.match(/step_\d+_preprocessed_(.+)\.[^.]+$/);
  if (!match) return filename;
  let name = match[1];
  // Strip known class-name suffixes
  name = name.replace(/(Preprocessor|Vectorizer|Generator|Filter|Splitter|Cleaner|Reducer|Optimizer)$/, "");
  // Insert space at camelCase boundaries (lowercase→uppercase or digit→uppercase) only
  name = name.replace(/([a-z0-9])([A-Z])/g, "$1 $2").trim();
  return name || match[1];
}

const VISUAL_EXTS = [".png", ".jpg", ".jpeg", ".svg"];
const GCODE_EXTS = [".gcode", ".optgcode"];

const StepViewerModal: React.FC<Props> = ({ isOpen, onClose, etchbotName, drawingStem }) => {
  const [steps, setSteps] = useState<StepFile[]>([]);
  const [loading, setLoading] = useState(false);
  const [selectedIdx, setSelectedIdx] = useState(0);

  useEffect(() => {
    if (!isOpen) return;
    setLoading(true);
    setSelectedIdx(0);
    axios.get<StepsResponse>(`${API_URL}/etchbot/${etchbotName}/drawing/${drawingStem}/steps`)
      .then(res => { setSteps(res.data.steps); setLoading(false); })
      .catch(() => { setSteps([]); setLoading(false); });
  }, [isOpen, etchbotName, drawingStem]);

  const selected = steps[selectedIdx];
  const imageUrl = selected
    ? `${API_URL}/etchbot/${etchbotName}/drawing/${drawingStem}/step_image/${encodeURIComponent(selected.filename)}`
    : null;
  const isVisual = selected ? VISUAL_EXTS.includes(selected.ext) : false;
  const isGcode = selected ? GCODE_EXTS.includes(selected.ext) : false;

  return (
    <Modal isOpen={isOpen} onClose={onClose} size="xl">
      <ModalOverlay />
      <ModalContent borderRadius="20px" overflow="hidden">
        <ModalHeader bg={colors.amber} color="rgba(0,0,0,0.7)" fontSize="13px" fontWeight="800">
          🖼️ {drawingStem} — Processing Steps
        </ModalHeader>
        <ModalCloseButton />
        <ModalBody p="16px">
          {loading ? (
            <Flex h="200px" align="center" justify="center"><Spinner color={colors.blue} /></Flex>
          ) : steps.length === 0 ? (
            <Text color="#888" fontSize="13px">No step images found. Processing output may have been cleared.</Text>
          ) : (
            <>
              {/* Step tabs */}
              <Flex gap="6px" flexWrap="wrap" mb="14px">
                {steps.map((step, i) => (
                  <Box
                    key={step.filename}
                    bg={i === selectedIdx ? colors.blue : "#f0f0f5"}
                    color={i === selectedIdx ? "white" : "#888"}
                    fontSize="9px"
                    fontWeight="700"
                    px="10px"
                    py="4px"
                    borderRadius="20px"
                    cursor="pointer"
                    onClick={() => setSelectedIdx(i)}
                    textTransform="uppercase"
                    letterSpacing="0.5px"
                  >
                    {formatStepName(step.filename)}
                  </Box>
                ))}
              </Flex>

              {/* Image viewer */}
              <Box
                bg="#f7f8fc"
                borderRadius="16px"
                minH="300px"
                display="flex"
                alignItems="center"
                justifyContent="center"
                overflow="hidden"
                position="relative"
                border="2px solid #eeeef5"
              >
                {imageUrl && isGcode ? (
                  <Box w="100%" h="100%" minH="340px">
                    <GCodeCanvas url={imageUrl} />
                  </Box>
                ) : imageUrl && isVisual && selected.ext === ".svg" ? (
                  <object data={imageUrl} type="image/svg+xml" style={{ maxWidth: "100%", maxHeight: "400px" }} />
                ) : imageUrl && isVisual ? (
                  <img src={imageUrl} alt={selected.filename} style={{ maxWidth: "100%", maxHeight: "400px", objectFit: "contain" }} />
                ) : (
                  <Box p="16px" textAlign="center">
                    <Text fontSize="12px" color="#888" mb="8px">File type not previewable</Text>
                    <Text fontSize="10px" color="#aaa">{selected?.filename}</Text>
                  </Box>
                )}

                {/* Prev / Next */}
                <Flex position="absolute" bottom="10px" right="10px" gap="6px">
                  <Button
                    size="sm"
                    borderRadius="50%"
                    bg={colors.blue}
                    color="white"
                    w="32px"
                    h="32px"
                    minW="32px"
                    p={0}
                    isDisabled={selectedIdx === 0}
                    onClick={() => setSelectedIdx(i => i - 1)}
                    _hover={{ opacity: 0.9 }}
                  >←</Button>
                  <Button
                    size="sm"
                    borderRadius="50%"
                    bg={colors.blue}
                    color="white"
                    w="32px"
                    h="32px"
                    minW="32px"
                    p={0}
                    isDisabled={selectedIdx === steps.length - 1}
                    onClick={() => setSelectedIdx(i => i + 1)}
                    _hover={{ opacity: 0.9 }}
                  >→</Button>
                </Flex>
              </Box>

              <Text color="#aaa" fontSize="9px" mt="6px" textAlign="right">{selected?.filename}</Text>
            </>
          )}
        </ModalBody>
      </ModalContent>
    </Modal>
  );
};

export default StepViewerModal;
