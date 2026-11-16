import fs from 'node:fs/promises';import {FileBlob,PresentationFile} from '@oai/artifact-tool';
const p=await PresentationFile.importPptx(await FileBlob.load('/home/duong-12a/fr3_hungros2/slides/.build/slide-1-5-v8.pptx'));
await fs.writeFile('/home/duong-12a/fr3_hungros2/slides/output/FR3_slide_05_v8.png',new Uint8Array(await(await p.export({slide:p.slides.items[4],format:'png',scale:1})).arrayBuffer()));console.log('rendered');
